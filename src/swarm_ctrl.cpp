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


void odom_sub_cb(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg = *msg;
}


void ctrl_timer_cb(const ros::TimerEvent &event)
{
    switch(node_id)
    {
        case 0:
        {
            //This is the leader, use a periodic function to generate the velocity from a trajectory
                
            static double t0 = ros::Time::now().toSec();
            static double t_prev = ros::Time::now().toSec() - t0;
            double t_curr = ros::Time::now().toSec() - t0;

            airsim_ros_pkgs::VelCmd vel_cmd;

            vel_cmd.twist.linear.x = 50*2*M_PI/150*cos( t_curr*2*M_PI/150 ); //Move back and forth along the 100 m road for 2.5 mins
            vel_cmd.twist.linear.y = 0.0;
            vel_cmd.twist.linear.z = 0.5*(-5 - odom_msg.pose.pose.position.z); //todo: use a proportional term to control the altitude

            vel_cmd.twist.angular.x = 0.0;
            vel_cmd.twist.angular.y = 0.0;
            vel_cmd.twist.angular.z = 0.0;

            vel_cmd_pub.publish(vel_cmd);

            break;
        }
        default:
        {
            //This is the leader, use a periodic function to generate the velocity from a trajectory
                
            static double t0 = ros::Time::now().toSec();
            static double t_prev = ros::Time::now().toSec() - t0;
            double t_curr = ros::Time::now().toSec() - t0;

            airsim_ros_pkgs::VelCmd vel_cmd;

            vel_cmd.twist.linear.x = 50*2*M_PI/150*cos( t_curr*2*M_PI/150 ); //Move back and forth along the 100 m road for 2.5 mins
            vel_cmd.twist.linear.y = 0.0;
            vel_cmd.twist.linear.z = 0.5*(-5 - odom_msg.pose.pose.position.z); //todo: use a proportional term to control the altitude

            vel_cmd.twist.angular.x = 0.0;
            vel_cmd.twist.angular.y = 0.0;
            vel_cmd.twist.angular.z = 0.0;

            vel_cmd_pub.publish(vel_cmd);

            break;
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_demo");
    ros::NodeHandle swarm_ctrl_nh("~");

    printf("Swarm control test.\n");

    /* #region - Node identity ----------------------------------------------------------------------------------------*/

    if(swarm_ctrl_nh.getParam("node_id", node_id))
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

    // Advertise the topic
    vel_cmd_pub = swarm_ctrl_nh.advertise<airsim_ros_pkgs::VelCmd>
                                    ("/airsim_node/drone_"
                                     + to_string(node_id)
                                     + "/vel_cmd_world_frame",
                                     1);

    //Subscribe to the takeoff service
    takeoff_srv_client = swarm_ctrl_nh.serviceClient<airsim_ros_pkgs::Takeoff>
                                        ("/airsim_node/drone_"
                                         + to_string(node_id)
                                         + "/takeoff"
                                        );
    // Collect the control rate parameter
    double ctrl_rate = 0;
    if( swarm_ctrl_nh.getParam("ctrl_rate", ctrl_rate) )
    {
        printf(KBLU "Node %d, control rate found: %.1f Hz\n" RESET, node_id, ctrl_rate);
    }
    else
    {
        printf(KRED "Node %d, control rate not found. Exiting\n" RESET, node_id);
        exit(-1);
    }

    //Create timer to calculate and publish control signal
    ros::Timer ctrl_timer = swarm_ctrl_nh.createTimer(ros::Duration(1/ctrl_rate), ctrl_timer_cb);
    ctrl_timer.stop();

    /* #endregion - Control related configurations ------------------------------------------------------------------------*/


    // Collecting the topic for feedback
    string odom_topic;
    if ( swarm_ctrl_nh.getParam("odom_topic", odom_topic) )
    {
        printf(KBLU "Node %d, odom topic found: %s\n" RESET, node_id, odom_topic.c_str());
    }
    else
    {
        printf(KRED "Node %d, odom topic not. Exiting!\n" RESET, node_id);
        exit(-1);
    }
    
    // Subsribing to the feedback topic
    ros::Subscriber odom_sub = swarm_ctrl_nh.subscribe(odom_topic, 50, odom_sub_cb);


    // Sleep for 5 seconds, take off, then sleep for 5 seconds
    ros::Duration(5).sleep();

    airsim_ros_pkgs::Takeoff takeoff_srv;

    takeoff_srv.request.waitOnLastTask = false;

    takeoff_srv_client.call(takeoff_srv);

    ros::Duration(5).sleep();

    printf("Node %d: Takeoff completed.\n", node_id);

    // Start the control timer
    ctrl_timer.start();

    // Start the callbacks
    ros::spin();

    return 0;
}