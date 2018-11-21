//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#define USE_SIM
#define MAP_WATCHDOG_TIME 10
#define TAGID 0

//Map Struct
typedef struct 
{
    geometry_msgs::Pose origin;
    float resolution;
    uint32_t width;
    uint32_t height;
    uint32_t size;
    std::vector<int8_t> data;
    bool valid;
} MapInfo_t;

//Global variables
ros::Publisher marker_pub;
MapInfo_t gridMap;
bool mapWatchDogCalled;
#ifdef USE_SIM
ros::Publisher pose_publisher;
tf::TransformBroadcaster *br;
tf::Transform *tform;

//Callback function for the Position topic (SIMULATION)
void sim_pose_callback(const gazebo_msgs::ModelStates& msg)
{
	//This function is called when a new position message is received
	geometry_msgs::PoseWithCovarianceStamped curpose;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id="/map";
	curpose.pose.pose.position = msg.pose[1].position;
	curpose.pose.pose.orientation = msg.pose[1].orientation;
	pose_publisher.publish(curpose);

	// send transform
	// br = new tf::TransformBroadcaster;
	// tform = new tf::Transform;
	// tform->setOrigin( tf::Vector3(msg.pose[1].position.x, msg.pose[1].position.y, 0) );
	// tf::Quaternion q;
	// q.setEulerZYX(tf::getYaw(msg.pose[1].orientation), 0, 0);
	// tform->setRotation( q );
	// *tform = tform->inverse();
	// br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), "base_footprint", "map"));
}
#endif

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

// Watch dog timer for the /map topic
// The program will exit if map is not received
void mapTimerCallback(const ros::TimerEvent& e)
{
    mapWatchDogCalled = true;
}

//Example of drawing a curve
void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);

}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    ROS_INFO("Received Map Message");
    //This function is called when a new map is received
    gridMap.resolution = msg.info.resolution;
    gridMap.width = msg.info.width;
    gridMap.height = msg.info.height;
    gridMap.origin = msg.info.origin;
    gridMap.size = gridMap.height*gridMap.width;
    gridMap.data.resize(gridMap.size);

    for (int it = 0; it < gridMap.size; it++)
    {
        gridMap.data[it] = msg.data[it];
        //print the map captured
        // if (gridMap.data[it] > 0 && it%gridMap.width != 0)
        //     printf("1");
        // else if (gridMap.data[it] > 0)
        //     printf("\n1");
        // else   
        //     printf("0");
    }
    gridMap.valid = true;
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    // Init MapInfo struct. Doing this before subscriber is initialized
    gridMap.valid = false;

    //Subscribe to the desired topics and assign callbacks
    ROS_INFO("MAIN ENTER");
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  
#ifdef USE_SIM
    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber sim_pose_sub = n.subscribe("/gazebo/model_states", 1, sim_pose_callback);
    pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/indoor_pos", 1, true);
#endif

    // Init the map watch dog flag
    mapWatchDogCalled = false;

    //Velocity control variable
    geometry_msgs::Twist vel;

    ros::Timer timer = n.createTimer(ros::Duration(MAP_WATCHDOG_TIME), mapTimerCallback);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
        // if the first map node has not arived, keep on looping
        if(!gridMap.valid)
        {
            if(mapWatchDogCalled)
            {
                ROS_INFO("/map topic not received in time.");
                ROS_INFO("EXITING!");
                return 0;
            }
            continue;
        }

    	//Main loop code goes here:
    	// vel.linear.x = 0.1; // set linear speed
    	// vel.angular.z = 0.3; // set angular speed


    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
