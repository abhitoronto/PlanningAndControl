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
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// STD libs
#include <stdlib.h>
#include <time.h>

#define USE_SIM
#define MAP_WATCHDOG_TIME 10
#define TAGID 0
#define NUM_NEAREST_NEIGHBOURS 7
#define NUM_RAND_POINTS 10

using namespace Eigen;

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

// 2D pose Struct
typedef struct
{
    double x;
    double y;
    double angle;
} pose2d_t;

//Global variables
ros::Publisher marker_pub;
MapInfo_t gridMap;
bool mapWatchDogCalled;
pose2d_t currPose;
std::vector<pose2d_t> goals;
int currGoal;

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

	currPose.x = msg.pose[1].position.x; // Robot X psotition
	currPose.y = msg.pose[1].position.y; // Robot Y psotition
 	currPose.angle = msg.pose[1].orientation.z; // Robot Yaw

	// send transform
	br = new tf::TransformBroadcaster;
	tform = new tf::Transform;
	tform->setOrigin( tf::Vector3(msg.pose[1].position.x, msg.pose[1].position.y, 0) );
	tf::Quaternion q;
	q.setEulerZYX(tf::getYaw(msg.pose[1].orientation), 0, 0);
	tform->setRotation( q );
	*tform = tform->inverse();
	br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), "base_footprint", "map"));

	double tempAngle = (tf::getYaw(msg.pose[1].orientation)); // Robot Yaw

	if (currPose.angle < 0)
	{
		ROS_INFO("Angle Wrap\n");
		currPose.angle = tempAngle + 6.2332;
	}
	else
	{
		currPose.angle = tempAngle;
	}
}
#endif

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	currPose.x = msg.pose.pose.position.x; // Robot X psotition
	currPose.y = msg.pose.pose.position.y; // Robot Y psotition
 	currPose.angle = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	// std::cout << "X: " << currPose.x
    //           << ", Y: " << currPose.y
    //           << ", Yaw: " << currPose.angle 
    //           << std::endl ;
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

void printMap(const MapInfo_t& map)
{
    for (uint32_t it = 0; it < gridMap.size; it++)
    {
        if (it%map.width == 0)
            printf("\n");
        if (map.data[it] > 0)
            printf("1");
        else   
            printf("0");
    }
    printf("\n");
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

    for (uint32_t it = 0; it < gridMap.size; it++)
    {
        gridMap.data[it] = msg.data[it];
    }
    gridMap.valid = true;
    //print the map captured
    printMap(gridMap);
}

void initGoals()
{
    // Let the current location be the first goal
    goals.push_back( currPose );
    pose2d_t tempPose;
    tempPose.x = 4.0f; tempPose.y = 0.f; tempPose.angle = 0.f;
    goals.push_back( tempPose );
    tempPose.x = 8.0f; tempPose.y = -4.0f; tempPose.angle = 3.14f;
    goals.push_back( tempPose );
    tempPose.x = 8.0f; tempPose.y = 0.f; tempPose.angle = -1.57f;
    goals.push_back( tempPose );

    // Initalize current goal number to be 0
    currGoal = 0;
}

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

//    ROS_INFO("running");
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

double get2dDistance(pose2d_t p1, pose2d_t p2)
{
    return sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) );
}

// p1 - p2
pose2d_t pose2dDiff(pose2d_t p1, pose2d_t p2)
{
    pose2d_t tempPose;
    tempPose.x = p1.x - p2.x;
    tempPose.y = p1.y - p2.y;
    tempPose.angle = 0;

    return tempPose;
}

//p1 + p2
pose2d_t pose2dSum(pose2d_t p1, pose2d_t p2)
{
    pose2d_t tempPose;
    tempPose.x = p1.x + p2.x;
    tempPose.y = p1.y + p2.y;
    tempPose.angle = 0;

    return tempPose;
}

pose2d_t getPoseFromMapPoint(const MapInfo_t& map, uint32_t pointIdx)
{
    pose2d_t returnPoint;
    if (pointIdx > map.size)
    {
        returnPoint.x = 0.f;
        returnPoint.y = 0.f;
        returnPoint.angle = 0.f;
        ROS_INFO("Pose index %d is out of bound %d", pointIdx, map.size);
        return returnPoint;
    }

    // Get x y coordinates of the map point in meters by applying the proper scaling
    double x_map = static_cast<double>(pointIdx % map.width) * map.resolution;
    double y_map = static_cast<double>(pointIdx / map.width) * map.resolution;

    // Offset the coordinates with the map origin
    x_map += map.origin.position.x;
    y_map += map.origin.position.y;

    returnPoint.x = x_map;
    returnPoint.y = y_map;
    returnPoint.angle = 0.f;

    return returnPoint;
}

uint32_t getMapPointFromPose(const MapInfo_t& map, pose2d_t pose)
{
    // Transform pose to get coordinates in the map axes
    double x_map = pose.x - map.origin.position.x;
    double y_map = pose.y - map.origin.position.y;

    // Compute the Map coordinates from point
    uint32_t x_idx = x_map / map.resolution;
    uint32_t y_idx = y_map / map.resolution;

    uint32_t map_idx = static_cast<uint32_t>(y_idx*map.width + x_idx);

    if (map_idx > map.size)
    {
        ROS_INFO("***ERROR*** : Point not present in the map");
        return 0;
    }

    return map_idx;
}

// Path planning function
// Inputs: current position, goal position, map
// Output: waypoints array
bool plan2dPath(const pose2d_t& currPose, 
                const pose2d_t& goalPose, 
                std::vector <pose2d_t>& wpArray,
                const MapInfo_t& map,
                uint32_t numRandPoints,
                uint32_t numNN)
{
    // Generate Random points in the map and store them
    std::vector<uint32_t> randWpIdx;
    std::vector<uint8_t> wpGrid(map.size, 0);
    
    // Starting point is the first waypoint
    uint32_t startIdx = getMapPointFromPose(map, currPose);
    randWpIdx.push_back(startIdx);
    wpGrid[startIdx] = 1;

    srand(time(NULL));
    for (uint32_t count = 0; count < numRandPoints; )
    {
        uint32_t randIdx = rand() % map.size;
        if (map.data[randIdx] > 0 || wpGrid[randIdx] > 0 )
        {
            continue;
        }
        randWpIdx.push_back(randIdx);
        wpGrid[randIdx] = 1;
        count++;
    }

    // Goal is the last waypoint 
    uint32_t goalIdx = getMapPointFromPose(map, goalPose);
    randWpIdx.push_back(goalIdx);
    wpGrid[goalIdx] = 1;

    // Print the wp map for debugging
    ROS_INFO("MAP of wayPoints");
    for (uint32_t it = 0; it < map.size; it++)
    {
        if (it%map.width == 0)
            printf("\n");
        if (wpGrid[it] + map.data[it]  > 0 )
            printf("1");
        else   
            printf("0");
    }
    printf ("\n");
    ROS_INFO("Num of random points %ld", randWpIdx.size());
    
    // Vector containing weights 
    std::vector<double> edgeWeights(randWpIdx.size()*randWpIdx.size(), 0.f);
    // iterate through each point and generate Graph
    for (uint32_t out_it = 0; out_it < randWpIdx.size(); out_it++)
    {
        pose2d_t curPose = getPoseFromMapPoint(map, randWpIdx[out_it]);

        // Vector containing weights temp
        std::vector<double> edgeWeightsTemp(randWpIdx.size(), 0.f);
        
        // Find distance with every single point
        for (uint32_t in_it = 0; in_it < randWpIdx.size(); in_it++)
        {
            if (in_it == out_it)
                continue;
            
            // find distance between p1 and p2
            pose2d_t in_pose = getPoseFromMapPoint(map, randWpIdx[in_it]);
            double distance = get2dDistance( in_pose, curPose);
            
            // Add this distance to edge matrix
            edgeWeightsTemp[in_it] = distance;
        }

        // finds a lowest points greater than current lowest
        double currentLowest = 0.f;
        for (uint32_t NNit = 0; NNit < numNN; NNit++)
        {
            double lowestDistance = 150.0;
            uint32_t lowestIdx = 0;
            // Each iteration goes through all indices and check if the current index
            // is the lowest while being greater than current lowest
            for (uint32_t in_it = 0; in_it < randWpIdx.size(); in_it++)
            {
                if (in_it == out_it)
                    continue;
                
                if(edgeWeightsTemp[in_it] <= lowestDistance &&
                   edgeWeightsTemp[in_it] > currentLowest)
                {
                    lowestIdx = in_it;
                    lowestDistance = edgeWeightsTemp[in_it];
                }
            }
            // Only the points that are found to be the lowest are put into this weights matrix
            edgeWeights[out_it*randWpIdx.size() + lowestIdx] = lowestDistance;
            currentLowest = lowestDistance;
        }
        // TODO: Check if the edges with weights don't collides with a obstacles
            // Bresenham stuff here
    }

    // Print the weights array
    for (uint32_t out_it = 0; out_it < randWpIdx.size(); out_it++)
    {
        printf("\n");
        for (uint32_t in_it = 0; in_it < randWpIdx.size(); in_it++)
        {
            printf(" %f", edgeWeights[out_it*randWpIdx.size() + in_it]);
        }
    }
    
    // Run A star on this graph
        // NEEDS:
            // matrix of edges with weights
            // Heuristic cost array
    return true;
}


void robotController(const pose2d_t& waypoint, geometry_msgs::Twist &move_cmd, float goal_angle)
{
	
        move_cmd.linear.x = 0;

	if (fabs(currPose.angle - goal_angle) < 0.1) 
	{
		move_cmd.angular.z = 0;
		return;
	}

	
	if ((currPose.angle - goal_angle) > 0.05) 
		move_cmd.angular.z = -0.05*fabs(currPose.angle - goal_angle);
	if ((currPose.angle - goal_angle) < 0.05) 
		move_cmd.angular.z = 0.05*fabs(currPose.angle - goal_angle);

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

    // Check for messages
    ros::spinOnce();

    // Init global variables
    mapWatchDogCalled = false;
    currPose.x = 0.0; currPose.y = 0.0; currPose.angle = 0.0;

    // Declare Local variables
    std::vector<pose2d_t> wayPoints;
    int currWp = 0;

    //Init goal locations
    initGoals();

    //Velocity control variable
    // geometry_msgs::Twist vel;

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

        // Get current goal
        // if robot has reached goal
        if( get2dDistance(goals[currGoal], currPose) <= 0.25 )
        {
            //if there is more targets in the goals array
            if(currGoal < goals.size()-1)
            {
                ROS_INFO("REACHED GOAL: %f,%f,%f", goals[currGoal].x,
                                                goals[currGoal].y,
                                                goals[currGoal].angle);
                // increment current goal
                currGoal++;
                ROS_INFO("NEXT GOAL: %f,%f,%f", goals[currGoal].x,
                                                goals[currGoal].y,
                                                goals[currGoal].angle); 
                
                // Run Planning function based on the map given
                wayPoints.clear();
                plan2dPath(currPose,
                           goals[currGoal], 
                           wayPoints, 
                           gridMap,
                           NUM_RAND_POINTS, 
                           NUM_NEAREST_NEIGHBOURS);
                currWp = 0;
            }

            // else break loop
            else
            {
                ROS_INFO("All Goals reached!");
                break;
            }
        }

        // Get current waypoint
        // if robot has reached waypoint
            // increment waypoint

        // Call controller with the current waypoint
            // Controller returns current velocity to be published
	pose2d_t samplePoint;
	samplePoint.x = 0;
	samplePoint.y = 4;
	
	geometry_msgs::Twist move_cmd;

	float goal_angle;
	if (samplePoint.x == currPose.x)
	{
		if (samplePoint.y > currPose.y)
			goal_angle = 1.57;
		else
			goal_angle = -1.57;
	}
	else
	{
		goal_angle = atan((samplePoint.y - currPose.y)/(samplePoint.x - currPose.x));
		//goal_angle = 1.57;
	}

	while (fabs(currPose.angle - goal_angle) > 0.1)
	{
		ros::spinOnce();   //Check for new messages
		robotController(samplePoint, move_cmd, goal_angle);
		velocity_publisher.publish(move_cmd); // Publish the command velocity
		float difference = fabs(currPose.angle - goal_angle);
		ROS_INFO("Current Angle: %f\n Angle Difference: %f\n", currPose.angle, difference);
		ros::spinOnce();
	}

	int stepsize = 100;
	float stepsizeFloat = float(stepsize);
	float x_dist = samplePoint.x - currPose.x;
	float y_dist = samplePoint.y - currPose.y;

	while(fabs(currPose.x - samplePoint.x) > 0.1 || fabs((currPose.y - samplePoint.y) > 0.1))
	{
		ros::spinOnce();   //Check for new messages
		move_cmd.linear.x = x_dist/stepsizeFloat;
		move_cmd.linear.y = y_dist/stepsizeFloat;
		velocity_publisher.publish(move_cmd); // Publish the command velocity
		ROS_INFO("X Velocity: %f\n Y Velocity: %f\n", move_cmd.linear.x, move_cmd.linear.y);
	}
	
	move_cmd.linear.x = 0;
	move_cmd.linear.y = 0;
	velocity_publisher.publish(move_cmd); // Publish the command velocity

        // Publish velocity that you get from controller
        //velocity_publisher.publish(move_cmd); // Publish the command velocity
    }

    return 0;
}
