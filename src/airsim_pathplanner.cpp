#include <stdio.h>
#include <vector>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include "uwb_driver/UwbRange.h"

// Airsim velocity message
#include <airsim_ros_pkgs/VelCmd.h>
// Airsim takeoff message
#include <airsim_ros_pkgs/Takeoff.h>

#include "math_common.h"

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

using namespace std;
using namespace Eigen;

/* #region - Network configuration ------------------------------------------------------------------------------------*/

// Numerical identity of the node
int node_id;

/* #endregion - Network configuration ---------------------------------------------------------------------------------*/


/* #region - Control configuration ------------------------------------------------------------------------------------*/

 ros::ServiceClient takeoff_srv_client;
 ros::Publisher vel_cmd_pub;

/* #endregion - Control configuration ---------------------------------------------------------------------------------*/


/* #region - Node's perception-----------------------------------------------------------------------------------------*/
nav_msgs::Odometry odom_msg;

/* #endregion - Node's perception--------------------------------------------------------------------------------------*/

/* #region - Mission */

vector<Vector3d> setpoints;
double sweep_velocity = 0.05;
double reach_tolerance = 0.2;
double Kp = 1.0;
double Kp_yaw = 0.25;
double yaw_rate_max = 0.1;
bool   fixed_yaw = false;

Vector3d odom_offset;

/* #endregion - Mission */

void odom_sub_cb(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg = *msg;

    static bool offset_saved = false;
    if (!offset_saved)
    {
        offset_saved = true;

        // This is needed because airsim has some offset in z
        // depending on the environment used
        odom_offset = Vector3d(odom_msg.pose.pose.position.x,
                               odom_msg.pose.pose.position.y,
                               odom_msg.pose.pose.position.z);

        printf("Odom offset registered: %f, %f, %f\n",
                odom_offset.x(),
                odom_offset.y(),
                odom_offset.z()
              ); 
    }
}


void ctrl_timer_cb(const ros::TimerEvent &event)
{
    // Process the feedback
    Vector3d curr_position = Vector3d(odom_msg.pose.pose.position.x,
                                      odom_msg.pose.pose.position.y,
                                      odom_msg.pose.pose.position.z) - odom_offset;

    double curr_roll, curr_pitch, curr_yaw;
    tf::Matrix3x3(tf::Quaternion(odom_msg.pose.pose.orientation.x,
                                 odom_msg.pose.pose.orientation.y,
                                 odom_msg.pose.pose.orientation.z,
                                 odom_msg.pose.pose.orientation.w)).getRPY(curr_roll, curr_pitch, curr_yaw);

    // Calculate the position and yaw setpoints
    static int curr_setpoint_idx = 0;
    // Postion:
    Vector3d curr_setpoint = setpoints[curr_setpoint_idx];
    // Yaw:
    double yaw_setpoint = curr_yaw;
    if(curr_setpoint_idx == 0)
        yaw_setpoint = curr_yaw;
    else
    {
        Vector3d dir = curr_setpoint - setpoints[curr_setpoint_idx - 1];
        if( sqrt(dir.x()*dir.x() + dir.y()*dir.y()) > 0.05 )
            yaw_setpoint = atan2(dir.y(), dir.x());
    }

    // Verify if the setpoints have been reached
    // Position:
    static vector<bool> position_setpoint_reached = vector<bool>(setpoints.size(), false);
    Vector3d position_error = curr_setpoint - curr_position;
    // If only current position setpoint has not been flagged as reached, then assess it
    if (!position_setpoint_reached[curr_setpoint_idx])
        position_setpoint_reached[curr_setpoint_idx] = (curr_position - curr_setpoint).norm() < reach_tolerance;
    // Yaw:
    static vector<bool> yaw_setpoint_reached = vector<bool>(setpoints.size(), false);
    double yaw_error = -math_common::angular_dist(yaw_setpoint, curr_yaw);
    if (!yaw_setpoint_reached[curr_setpoint_idx])
    {
        if(curr_setpoint_idx == 0)
            yaw_setpoint_reached[curr_setpoint_idx] = true;
        else
        {
            Vector3d dir = curr_setpoint - setpoints[curr_setpoint_idx - 1];
            if( sqrt(dir.x()*dir.x() + dir.y()*dir.y()) < 0.025 || fabs(yaw_error) < 0.1 )
                yaw_setpoint_reached[curr_setpoint_idx] = true;
            else
                yaw_setpoint_reached[curr_setpoint_idx] = false; 
        }
    }

    // Generate twist
    Vector3d linear_vel(0, 0, 0);
    double yaw_rate = 0;
    // If current setpoint has not reached the last, then proceed as normal.
    if ( curr_setpoint_idx < setpoints.size() )
    {
        ROS_INFO_THROTTLE(1.0, "sp: %d, yaw_sp_r: %s. yaw_sp: %.4f. yaw_sp_err: %.4f. "
                         "pos_sp_r: %s. pos: %.2f, %.2f, %.2f. pos_err: %.2f, %.2f, %.2f\n",
                          curr_setpoint_idx,
                          yaw_setpoint_reached[curr_setpoint_idx] ? "true" : "false",
                          yaw_setpoint,
                          yaw_error,
                          position_setpoint_reached[curr_setpoint_idx] ? "true" : "false",
                          curr_setpoint.x(),
                          curr_setpoint.y(),
                          curr_setpoint.z(),
                          position_error.x(),
                          position_error.y(),
                          position_error.z());

        // If yaw setpoint has not been reached, don't generate the linear velocity
        if ( !yaw_setpoint_reached[curr_setpoint_idx] && !fixed_yaw )
        {
            yaw_rate = Kp_yaw*yaw_error;
            yaw_rate = yaw_rate_max/max(yaw_rate_max,fabs(yaw_rate))*yaw_rate;
            linear_vel = Vector3d(0, 0, 0);
        }
        else if ( !position_setpoint_reached[curr_setpoint_idx] )
        {
            if (fixed_yaw)
                yaw_rate = 0;
            else
            {
                yaw_rate = Kp_yaw*yaw_error;
                yaw_rate = yaw_rate_max/max(yaw_rate_max,fabs(yaw_rate))*yaw_rate;
            }
            linear_vel = Kp*position_error;
            linear_vel = sweep_velocity/max(sweep_velocity, linear_vel.norm())*linear_vel;
        }
        else
        {
            curr_setpoint_idx++;
            if (curr_setpoint_idx < setpoints.size())        
                printf("Reached: Setpoint %d [%6.2f, %6.2f, %6.2f].\n"
                        "  Next: Setpoint %d [%6.2f, %6.2f, %6.2f].\n",
                        curr_setpoint_idx-1,
                        setpoints[curr_setpoint_idx-1].x(),
                        setpoints[curr_setpoint_idx-1].y(),
                        setpoints[curr_setpoint_idx-1].z(),
                        curr_setpoint_idx,
                        setpoints[curr_setpoint_idx].x(),
                        setpoints[curr_setpoint_idx].y(),
                        setpoints[curr_setpoint_idx].z());
            else
            {
                printf("Reached: Setpoint %d [%6.2f, %6.2f, %6.2f].\n"
                        "  Next: Finished.\n",
                        curr_setpoint_idx-1,
                        setpoints[curr_setpoint_idx-1].x(),
                        setpoints[curr_setpoint_idx-1].y(),
                        setpoints[curr_setpoint_idx-1].z(),
                        curr_setpoint_idx,
                        setpoints[curr_setpoint_idx].x(),
                        setpoints[curr_setpoint_idx].y(),
                        setpoints[curr_setpoint_idx].z());
            }
            
            return;
        }

        airsim_ros_pkgs::VelCmd vel_cmd;

        vel_cmd.twist.linear.x = linear_vel.x();
        vel_cmd.twist.linear.y = linear_vel.y();
        vel_cmd.twist.linear.z = linear_vel.z();
        
        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = yaw_rate;

        vel_cmd_pub.publish(vel_cmd);
    }
    else // Current setpoint is the last one, publish (0, 0, 0) velocity to land
    {
        airsim_ros_pkgs::VelCmd vel_cmd;

        vel_cmd.twist.linear.x =  0.0;
        vel_cmd.twist.linear.y =  0.0;
        vel_cmd.twist.linear.z = -sweep_velocity;

        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = 0.0;

        vel_cmd_pub.publish(vel_cmd);
    }
    

    static bool shutdown_start = false;
    if(curr_setpoint_idx == setpoints.size()-1 || shutdown_start)
    {
        static ros::Time shutdown_time = ros::Time::now();
        shutdown_start = true;
        if( (ros::Time::now() - shutdown_time).toSec() > 20 )
            exit(-1);
    }

    // switch(node_id)
    // {
    //     case 0:
    //     {
    //         //This is the leader, use a periodic function to generate the velocity from a trajectory
                
    //         static double t0 = ros::Time::now().toSec();
    //         static double t_prev = ros::Time::now().toSec() - t0;
    //         double t_curr = ros::Time::now().toSec() - t0;

    //         airsim_ros_pkgs::VelCmd vel_cmd;

    //         vel_cmd.twist.linear.x = 0*5*2*M_PI/150*cos( t_curr*2*M_PI/150 ); //Move back and forth along the 100 m road for 2.5 mins
    //         vel_cmd.twist.linear.y = 0.0;
    //         vel_cmd.twist.linear.z = 0.5*(-5 - odom_msg.pose.pose.position.z); //todo: use a proportional term to control the altitude

    //         vel_cmd.twist.angular.x = 0.0;
    //         vel_cmd.twist.angular.y = 0.0;
    //         vel_cmd.twist.angular.z = 0.0;

    //         vel_cmd_pub.publish(vel_cmd);

    //         break;
    //     }
    //     default:
    //     {
    //         //This is the leader, use a periodic function to generate the velocity from a trajectory
                
    //         static double t0 = ros::Time::now().toSec();
    //         static double t_prev = ros::Time::now().toSec() - t0;
    //         double t_curr = ros::Time::now().toSec() - t0;

    //         airsim_ros_pkgs::VelCmd vel_cmd;

    //         vel_cmd.twist.linear.x = 0*50*2*M_PI/150*cos( t_curr*2*M_PI/150 ); //Move back and forth along the 100 m road for 2.5 mins
    //         vel_cmd.twist.linear.y = 0.0;
    //         vel_cmd.twist.linear.z = 0.5*(-5 - odom_msg.pose.pose.position.z); //todo: use a proportional term to control the altitude

    //         vel_cmd.twist.angular.x = 0.0;
    //         vel_cmd.twist.angular.y = 0.0;
    //         vel_cmd.twist.angular.z = 0.0;

    //         vel_cmd_pub.publish(vel_cmd);

    //         break;
    //     }
    // }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_pathplanner");
    ros::NodeHandle nh("~");

    printf("Swarm control test.\n");

    /* #region - Node identity ----------------------------------------------------------------------------------------*/

    if(nh.getParam("node_id", node_id))
    {
        printf(KBLU "node_id: %d\n" RESET, node_id);
    }
    else
    {
        printf(KRED "node_id not found. Exiting\n" RESET);
        exit(-1);
    }
    
    /* #endregion - Node identity -------------------------------------------------------------------------------------*/


    /* #region - Control related configurations ------------------------------------------------------------------------*/

    if(nh.getParam("fixed_yaw", fixed_yaw))
    {
        printf("Obtained value '%s' for param fixed_yaw\n", fixed_yaw? "true": "false");
    }
    else
    {
        fixed_yaw = false;
        printf("fixed_yaw not set. Use 'false' by default\n");
    }
    

    // Advertise the topic
    vel_cmd_pub = nh.advertise<airsim_ros_pkgs::VelCmd>
                                    ("/airsim_node/drone_"
                                     + to_string(node_id)
                                     + "/vel_cmd_world_frame",
                                     1);

    //Subscribe to the takeoff service
    takeoff_srv_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>
                                        ("/airsim_node/drone_"
                                         + to_string(node_id)
                                         + "/takeoff"
                                        );
    // Collect the control rate parameter
    double ctrl_rate = 0;
    if( nh.getParam("ctrl_rate", ctrl_rate) )
    {
        printf(KBLU "Node %d, control rate found: %.1f Hz\n" RESET, node_id, ctrl_rate);
    }
    else
    {
        printf(KRED "Node %d, control rate not found. Exiting\n" RESET, node_id);
        exit(-1);
    }

    //Create timer to calculate and publish control signal
    ros::Timer ctrl_timer = nh.createTimer(ros::Duration(1/ctrl_rate), ctrl_timer_cb);
    ctrl_timer.stop();

    /* #endregion - Control related configurations ------------------------------------------------------------------------*/


    /* #region - Collecting the sweeping mission -----------------------------------------------------------------------*/
    
    vector<double> setpoints_;
    if( nh.getParam("setpoints", setpoints_) )
    {
        printf(KBLU " %d setpoints found\n" RESET, setpoints_.size()/3);
    }
    else
    {
        printf(KYEL " No setpoint found. Staying fixed\n" RESET);
        setpoints_.push_back(0);
        setpoints_.push_back(0); 
        setpoints_.push_back(0);
    }

    for(int i = 0; i < setpoints_.size()/3; i++)
    {
        Vector3d setpoint;
        setpoint << setpoints_[i*3], setpoints_[i*3 + 1], setpoints_[i*3 + 2];
        setpoints.push_back(setpoint);
    }

    if( nh.getParam("sweep_velocity", sweep_velocity) )
    {
        printf(KBLU "sweep velocity found: %f\n" RESET, sweep_velocity);
    }
    else
    {
        sweep_velocity = 0.5;
        printf(KRED "sweep velocity not found. Using %f as default\n" RESET, sweep_velocity);   
    }
    
    if( nh.getParam("reach_tolerance", reach_tolerance) )
    {
        printf(KBLU "reach tolerance found: %f\n" RESET, reach_tolerance);
    }
    else
    {
        reach_tolerance = 0.2;
        printf(KRED "reach tolerance not found. Using %f as default\n" RESET, reach_tolerance);   
    }

    /* #endregion - Collecting the sweeping mission --------------------------------------------------------------------*/


    /* #region - Collecting the topic for feedback ---------------------------------------------------------------------*/
    
    string odom_topic;
    if ( nh.getParam("odom_topic", odom_topic) )
    {
        printf(KBLU "Node %d, odom topic found: %s\n" RESET, node_id, odom_topic.c_str());
    }
    else
    {
        printf(KRED "Node %d, odom topic not. Exiting!\n" RESET, node_id);
        exit(-1);
    }
    
    // Subsribing to the feedback topic
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 50, odom_sub_cb);

    /* #endregion - Collecting the topic for feedback ------------------------------------------------------------------*/


    // Spin for 10 seconds to initialize other variables,
    // then start the mission
    ros::Rate rate(20);
    ros::Time start_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - start_time).toSec() < 10.0 ) 
    {
        rate.sleep();
        ros::spinOnce();
    }

    // Wait for about 20 seconds to keep the processes cool
    ros::Duration(20).sleep();

    printf("Taking off.\n");

    airsim_ros_pkgs::Takeoff takeoff_srv;

    takeoff_srv.request.waitOnLastTask = false;

    takeoff_srv_client.call(takeoff_srv);

    ros::Duration(5).sleep();

    printf("Mission started.\n");

    // Start the control timer
    ctrl_timer.start();

    // Start the callbacks
    ros::spin();

    return 0;
}