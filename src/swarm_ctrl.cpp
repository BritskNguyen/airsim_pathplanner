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

/* #region - Control configuration ------------------------------------------------------------------------------------*/

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


    /* #region - Control related parameters ---------------------------------------------------------------------------*/
    
    // // Find the control topic
    // string vel_cmd_topic;
    // if(swarm_ctrl_nh.getParam("vel_cmd_topic", vel_cmd_topic))
    // {
    //     printf(KBLU "Node %d, velocity command topic found: %s\n" RESET, node_id, vel_cmd_topic);
    // }
    // else
    // {
    //     printf(KRED "Node %d, velocity command topic not found. Exiting\n" RESET, node_id);
    //     exit(-1);
    // }

    // Advertise the topic
    ros::Publisher vel_cmd_pub = swarm_ctrl_nh.advertise<airsim_ros_pkgs::VelCmd>
                                    ("/airsim_node/drone_"
                                     + to_string(node_id)
                                     + "/vel_cmd_world_frame",
                                     1);

    // Find the takeoff service
    // string takeoff_srv;
    // if(swarm_ctrl_nh.getParam("takeoff_srv", takeoff_srv))
    // {
    //     printf(KBLU "Node %d, take off service found: %s\n" RESET, node_id, takeoff_srv);
    // }
    // else
    // {
    //     printf(KRED "Node %d, take off service not found. Exiting\n" RESET, node_id);
    //     exit(-1);
    // }

    //Subscribe to the takeoff service
    takeoff_srv_client = swarm_ctrl_nh.serviceClient<airsim_ros_pkgs::Takeoff>
                                        ("/airsim_node/drone_"
                                         + to_string(node_id)
                                         + "/takeoff"
                                        );

    /* #endregion - Control related parameters ------------------------------------------------------------------------*/


    // Sleep for 5 seconds, take off, then sleep for 5 seconds
    ros::Duration(5).sleep();

    airsim_ros_pkgs::Takeoff takeoff_srv;

    takeoff_srv.request.waitOnLastTask = false;

    takeoff_srv_client.call(takeoff_srv);
    printf("Takeoff called.\n");

    ros::Duration(5).sleep();

    // Move forward for 20 seconds
    ros::Rate vel_cmd_rate(20);
    airsim_ros_pkgs::VelCmd vel_cmd;
    ros::Time start_time = ros::Time::now();

    vel_cmd.twist.linear.x = 5.0;
    vel_cmd.twist.linear.y = 0.0;
    vel_cmd.twist.linear.z = 0.0;

    vel_cmd.twist.angular.x = 0;
    vel_cmd.twist.angular.y = 0;
    vel_cmd.twist.angular.z = 0;

    vel_cmd_pub.publish(vel_cmd);

    while(ros::ok())
    {
        double vel_cmd_time = (ros::Time::now() - start_time).toSec();
        printf("velocity command sent. %f\n", vel_cmd_time);
        
        if( vel_cmd_time > 20 )
            break;

        // vel_cmd_pub.publish(vel_cmd);

        vel_cmd_rate.sleep();
    }

    // Start the callbacks
    ros::spin();

    return 0;
}