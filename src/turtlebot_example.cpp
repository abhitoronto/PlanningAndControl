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
// Eigen Specific includes
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// STD libs
#include <stdlib.h>
#include <time.h>
#include <algorithm>

// Definitions
#define USE_SIM
#define MAP_WATCHDOG_TIME 10
#define TAGID 0
#define NUM_NEAREST_NEIGHBOURS 8
#define NUM_RAND_POINTS 25
#define MAX_EDGE_DIST 50

// Colors
#define PURPLE 1,1 //purple
#define RED 1,0 //red
#define BLUE 0,1 //blue
#define BLACK 0,0 //black

using namespace Eigen;

// Map Struct
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
	currPose.x = msg.pose.pose.position.x; // Robot X psotition
	currPose.y = msg.pose.pose.position.y; // Robot Y psotition
 	currPose.angle = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	// std::cout << "X: " << currPose.x
    //           << ", Y: " << currPose.y
    //           << ", Yaw: " << currPose.angle 
    //           << std::endl;
}

// Watch dog timer for the /map topic
// The program will exit if map is not received
void mapTimerCallback(const ros::TimerEvent& e)
{
    mapWatchDogCalled = true;
}

//Example of drawing a curve
void drawPoints(std::vector<pose2d_t> points, double colourR, double colourB) 
{
    double steps = points.size();
    volatile static int k = 0;

    visualization_msgs::Marker visPoints;
    visPoints.header.frame_id = "/map";
    visPoints.id = k++; //each curve must have a unique id or you will overwrite an old ones
    visPoints.type = visualization_msgs::Marker::POINTS;
    visPoints.action = visualization_msgs::Marker::ADD;
    visPoints.ns = "point";
    visPoints.scale.x = 0.1;
    visPoints.scale.y = 0.1;
    visPoints.color.r = colourR;
    visPoints.color.b = colourB;
    visPoints.color.a = 1.0;

    //generate curve points
    for(int i = 0; i < steps; i++) {
        geometry_msgs::Point p;
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = 0; //not used
        visPoints.points.push_back(p); 
    }

    //publish new curve
    marker_pub.publish(visPoints);
}

//Example of drawing a line
void drawLine(pose2d_t p1, pose2d_t p2, double colourR, double colourB) 
{
    // Curves are drawn as a series of stright lines
    // Simply sample your curves into a series of points

    double ax = p1.x;
    double ay = p1.y;
    double bx = p2.x;
    double by = p2.y;
    volatile static int k = 0;

    visualization_msgs::Marker lines;
    lines.header.frame_id = "/map";
    lines.id = k++; //each curve must have a unique id or you will overwrite an old ones
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.action = visualization_msgs::Marker::ADD;
    lines.ns = "line";
    lines.scale.x = 0.01;
    lines.color.r = colourR;
    lines.color.b = colourB;
    lines.color.a = 1.0;

    //generate curve points
    // for(int i = 0; i < steps; i++) {
    geometry_msgs::Point p;
    p.x = ax;
    p.y = ay;
    p.z = 0; //not used
    lines.points.push_back(p);
    p.x = bx;
    p.y = by;
    p.z = 0; //not used
    lines.points.push_back(p);

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
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) 
{

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

bool checkEdgeCollision(const MapInfo_t& map,
                        uint32_t idxA,
                        uint32_t idxB)
{
    int ax = idxA % map.width;
    int ay = idxA / map.width;

    int bx = idxB % map.width;
    int by = idxB / map.width;

    std::vector<int> line_x;
    std::vector<int> line_y;

    // get the line points between A and B
    bresenham(ax, ay, bx, by, line_x, line_y);

    for(uint32_t p = 0; p < line_x.size() && p < line_y.size(); p++)
    {
        uint32_t map_idx = line_y[p]*map.width + line_x[p];
        if( map_idx < map.size)
        {
            if(map.data[map_idx] > 0)
            {
                // DEBUG: uncomment to see collisions.
                // drawLine(getPoseFromMapPoint(map,idxA), 
                //          getPoseFromMapPoint(map,map_idx),
                //          BLACK);
                return true;
            }
        }
    }
    return false;
}

typedef struct 
{
    uint32_t idx;
    double g;
    double h;
    uint32_t prev;
} SetItem_t;

// A-star implementation
//      The first point is the starting point
//      The last point is the goal
bool FindShortestPath( const std::vector<pose2d_t>& points, 
                       const MatrixXi & validEdge,
                       std::vector <pose2d_t>& wpArray)
{
    uint32_t start = 0;
    uint32_t end = points.size()-1;

    std::vector<SetItem_t> openSet;
    std::vector<SetItem_t> closedSet;

    SetItem_t elem;
    elem.idx = start;
    elem.prev = start;
    elem.g = 0.f;
    elem.h = get2dDistance(points[elem.idx],points[end]);

    // 1 Put node_start in the OPEN list with f(node_start) = h(node_start) (initialization)
    openSet.push_back(elem);

// 2 while the OPEN list is not empty {
    while (openSet.size() > 0)
    {
// 3 Take from the open list the node node_current with the lowest
        uint32_t lowest = -1;
        double currentLowest = MAX_EDGE_DIST;
        for(uint32_t i = 0; i < openSet.size(); i++)
        {
// 4 f(node_current) = g(node_current) + h(node_current)
            double cost = openSet[i].g + openSet[i].h;
            if (cost < currentLowest )
            {
                currentLowest = cost;
                lowest = i;
            }
        }
        if (lowest == -1)
            return false;
        
// 5 if node_current is node_goal we have found the solution; break
        if(openSet[lowest].idx == end)
        {
            closedSet.push_back(openSet[lowest]);
            break;
        }

// 6 Generate each state node_successor that come after node_current
        for(uint32_t i = 0; i < points.size(); i++)
        {
// 7 for each node_successor of node_current {
            if(validEdge(openSet[lowest].idx,i) == 1)
            {
// 8 Set successor_current_cost = g(node_current) + w(node_current, node_successor)
                SetItem_t curElem;
                curElem.idx = i;
                curElem.prev = openSet[lowest].idx;
                curElem.g = openSet[lowest].g + get2dDistance(points[curElem.idx],points[openSet[lowest].idx]);
                curElem.h = get2dDistance(points[curElem.idx],points[end]);

                int openIdx = -1; 
                for(uint32_t it = 0; it < openSet.size(); it++)
                {
                    if(openSet[it].idx == curElem.idx)
                        openIdx = it;
                }

                int closedIdx = -1;
                if (!closedSet.empty()) 
                {
                    for(uint32_t it = 0; it < closedSet.size(); it++)
                    {
                        if(closedSet[it].idx == curElem.idx)
                            closedIdx = it;
                    }
                }

// 9 if node_successor is in the OPEN list {
                if(openIdx >= 0) // CurElem.idx is in open set already
                {
// 10 if g(node_successor) ≤ successor_current_cost continue (to line 20)
                    if(openSet[openIdx].g < curElem.g)
                        continue;
                    openSet.erase(openSet.begin()+openIdx);
                    openSet.push_back(curElem);
                }
// 11 } else if node_successor is in the CLOSED list {
                else if(closedIdx >=0) // CurElem.idx is in closed set already
                {
// 12 if g(node_successor) ≤ successor_current_cost continue (to line 20)
                    if(closedSet[closedIdx].g < curElem.g)
                        continue;
// 13 Move node_successor from the CLOSED list to the OPEN list
                    openSet.push_back(curElem);
                    closedSet.erase(closedSet.begin()+closedIdx);
                }
// 14 } else {
                else
                {
// 15 Add node_successor to the OPEN list
                    openSet.push_back(curElem);       
                }
            }
        }
// 21 Add node_current to the CLOSED list
        ROS_INFO("E: %d %d %f", openSet[lowest].idx, openSet[lowest].prev, openSet[lowest].g);
        closedSet.push_back(openSet[lowest]);
        openSet.erase(openSet.begin()+lowest);
    }

// 23 if(node_current != node_goal) exit with error (the OPEN list is empty)
    if(closedSet.back().idx != end)
    {
        ROS_INFO("***ERROR***: A STAR did not find a path");
        return false;
    }
    else
    {
        wpArray.push_back(points[end]);
        SetItem_t curItem = closedSet.back();
        while(curItem.prev != start)
        {
            for(uint32_t i = 0; i < closedSet.size(); i++)
            {
                if(closedSet[i].idx == curItem.prev)
                {
                    curItem = closedSet[i];
                    break;
                }
            }
            wpArray.insert(wpArray.begin(), points[curItem.idx]);
        }
    }
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
    std::vector<uint32_t> randWpArray;
    std::vector<pose2d_t> randWpPoseArray;
    std::vector<uint8_t> wpGrid(map.size, 0);
    
    // Starting point is the first waypoint
    randWpPoseArray.push_back(currPose);
    uint32_t startIdx = getMapPointFromPose(map, currPose);
    randWpArray.push_back(startIdx);
    wpGrid[startIdx] = 1;

    srand(time(NULL));
    for (uint32_t count = 0; count < numRandPoints; )
    {
        uint32_t randIdx = rand() % map.size;
        if (map.data[randIdx] > 0 || wpGrid[randIdx] > 0 )
        {
            continue;
        }
        randWpArray.push_back(randIdx);
        wpGrid[randIdx] = 1;

        randWpPoseArray.push_back(getPoseFromMapPoint(map,randIdx));
        count++;
    }

    // Goal is the last waypoint 
    randWpPoseArray.push_back(goalPose);
    uint32_t goalIdx = getMapPointFromPose(map, goalPose);
    randWpArray.push_back(goalIdx);
    wpGrid[goalIdx] = 1;

    // Visualize the points on rviz
    drawPoints(randWpPoseArray, PURPLE);

    // // Print the wp map for debugging
    // ROS_INFO("MAP of WayPoints");
    // for (uint32_t it = 0; it < map.size; it++)
    // {
    //     if (it%map.width == 0)
    //         printf("\n");
    //     if (wpGrid[it]  > 0 )
    //         printf("2");
    //     else if ( map.data[it] > 0)
    //         printf("1");
    //     else   
    //         printf("0");
    // }
    // printf ("\n");
    ROS_INFO("Num of random points %ld", randWpArray.size());
    
    // Matrix containing edge weights NxN. Corresponds to the distance between inTH and outTH points.
    MatrixXd edgeWeights(randWpArray.size(), randWpArray.size());
    // Matrix showing edge connections
    MatrixXi edgeValid(randWpArray.size(), randWpArray.size());

// iterate through each point and generate Graph
    for (uint32_t out_it = 0; out_it < randWpArray.size(); out_it++)
    {
        pose2d_t curPose = randWpPoseArray[out_it];

        // Vector containing weights for only this iteration
        std::vector<double> edgeWeightsTemp(randWpArray.size(), MAX_EDGE_DIST);
        std::vector<int8_t> edgeValidTemp(randWpArray.size(), 0);
        
        // Create an edge weight matrix and check for edge collisions
        for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
        {
            // init the edge matrices with default values
            edgeWeights(out_it, in_it) = MAX_EDGE_DIST;
            edgeValid(out_it, in_it) = 0;

            if (in_it == out_it)
                continue;

            // find distance between p1 and p2
            pose2d_t in_pose = randWpPoseArray[in_it];
            double distance = get2dDistance( in_pose, curPose);
            
            // Add this distance to edge matrix
            edgeWeightsTemp[in_it] = distance;
        }

        // TODO: Check if the edges with weights don't collides with a obstacles
            // Bresenham stuff here
        for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
        {
            if (in_it == out_it)
                continue;
            if (!checkEdgeCollision(map, randWpArray[in_it], randWpArray[out_it]))
            {
                edgeValidTemp[in_it] = 1;
            }
        }

        // for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
        // {
        //     if (in_it == out_it)
        //         continue;
        //     edgeValid(out_it, in_it) = edgeValidTemp[in_it];
        // }

        // finds the closest numNN valid points to the current wp
        double currentLowest = 0.f;
        uint32_t currentLowestIdx = -1;
        for (uint32_t NNit = 0; NNit < numNN; NNit++)
        {
            double lowestDistance = MAX_EDGE_DIST;
            uint32_t lowestIdx = currentLowestIdx;
            bool foundLowest = false;
            // Each iteration goes through all indices and check if the current index
            // is the lowest while being greater than previous lowest
            for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
            {
                if (in_it == out_it || !edgeValidTemp[in_it])
                    continue;
                
                if(edgeWeightsTemp[in_it] <= lowestDistance &&
                   edgeWeightsTemp[in_it] >= currentLowest && 
                   in_it != currentLowestIdx)
                {
                    foundLowest = true;
                    lowestIdx = in_it;
                    lowestDistance = edgeWeightsTemp[in_it];
                }
            }
            // Only the valid edges that are found to be the lowest are put into this weights matrix
            if (foundLowest)
            {
                edgeWeights(out_it, lowestIdx) = lowestDistance;
                edgeValid(out_it, lowestIdx) = 1;
                currentLowestIdx = lowestIdx;
                currentLowest = lowestDistance;
            }
        }
    }

    // Print the weights array
    for (uint32_t out_it = 0; out_it < randWpArray.size(); out_it++)
    {
        printf("\n");
        for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
        {
            printf(" %2.2f", edgeWeights(out_it, in_it));
        }
    }
    printf("\n");

    // Print the valid array
    for (uint32_t out_it = 0; out_it < randWpArray.size(); out_it++)
    {
        printf("\n");
        for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
        {
            printf(" %d", edgeValid(out_it, in_it));
        }
    }
    printf("\n");

    // VISUALIZE valid edges
    // for (uint32_t out_it = 0; out_it < randWpArray.size(); out_it++)
    // {
    //     int count = 0;
    //     for (uint32_t in_it = 0; in_it < randWpArray.size(); in_it++)
    //     {
    //         if (edgeValid(out_it, in_it))
    //         {
    //             drawLine(randWpPoseArray[out_it],
    //                      randWpPoseArray[in_it],
    //                      RED);
    //             count++;
    //         }
    //     }
    //     printf("%d: %d\n", out_it, count);
    // }

    // Run A star on this graph
    return FindShortestPath(randWpPoseArray, edgeValid, wpArray);
    // return true;
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
                //TODO: put this inside plan2dPath if statement
                // increment current goal
                currGoal++;

                // Run Planning function based on the map given
                wayPoints.clear();
                if (plan2dPath(currPose,
                           goals[currGoal], 
                           wayPoints, 
                           gridMap,
                           NUM_RAND_POINTS, 
                           NUM_NEAREST_NEIGHBOURS))
                {
                    ROS_INFO("NEXT GOAL: %f,%f,%f", goals[currGoal].x,
                                                    goals[currGoal].y,
                                                    goals[currGoal].angle); 
                }

                currWp = 0;
            }

            // else break loop
            else
            {
                ROS_INFO("All Goals reached!");
                break;
            }
        }

        // Visualize all waypoints
        drawLine(currPose, wayPoints[currWp], BLUE);
        for(uint32_t i = currWp; i < wayPoints.size()-1; i++)
        {
            drawLine(wayPoints[i], wayPoints[i+1], BLUE);
        }

        // Get current waypoint
        // if robot has reached waypoint
            // increment waypoint

        // Call controller with the current waypoint
            // Controller returns current velocity to be published

        // Publish velocity that you get from controller
        // velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
