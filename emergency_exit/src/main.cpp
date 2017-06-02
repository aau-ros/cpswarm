#include <ros/ros.h>
#include <ros/console.h>
//#include <emergency_exit.h>
//#include "cv.h"
//#include "ml.h"
#include <ros/service_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include <array>
#include <math.h>
#include <sstream>
#include <fstream>

// http://docs.opencv.org/3.2.0/d0/dce/classcv_1_1ml_1_1ANN__MLP.html

using namespace std;

/**
 * Agent position.
 */
geometry_msgs::Pose pose;
bool pose_valid = false;

/**
 * Grid map.
 */
nav_msgs::OccupancyGrid grid_map;
bool map_valid = false;

/**
 * Log file.
 */
string log_file;
bool log_valid = false;
fstream fs_log;

/**
 * Simulated discrete time step.
 */
int step;

/**
 * Distance (in x and y direction) that the agent can move in each step (in meters).
 */
double step_size;

/**
 * Closest emergency exit.
 */
double closest_dist;

/**
 * Convert given relative coordinates into a direction.
 * @param int* dir: Pointer to the direction.
 *                  3 2 1
 *                  4   0
 *                  5 6 7
 * @param int* dx: Pointer to the x coordinate.
 * @param int* dy: Pointer to the y coordinate.
 * @return bool: Success of conversion.
 */
bool coord2dir(int* dir, int* dx, int* dy)
{
    if(*dx > 0 && *dy == 0)
        *dir = 0;
    else if(dx > 0 && *dy > 0)
        *dir = 1;
    else if(*dx == 0 && *dy > 0)
        *dir = 2;
    else if(*dx < 0 && *dy > 0)
        *dir = 3;
    else if(*dx < 0 && *dy == 0)
        *dir = 4;
    else if(*dx < 0 && *dy < 0)
        *dir = 5;
    else if(*dx == 0 && *dy < 0)
        *dir = 6;
    else if(*dx > 0 && *dy < 0)
        *dir = 7;
    else{
        ROS_ERROR("Invalid relative coordinates (%d,%d)!", *dx, *dy);
        return false;
    }
    
    return true;
}

/**
 * Convert a given direction to relative coordinates.
 * @param int* dir: Pointer to the direction.
 *                  3 2 1
 *                  4   0
 *                  5 6 7
 * @param int* dx: Pointer to the x coordinate.
 * @param int* dy: Pointer to the y coordinate.
 * @return bool: Success of conversion.
 */
bool dir2coord(int* dir, int* dx, int* dy)
{
    switch(*dir){
        case 0:
            *dx = 1;
            *dy = 0;
            break;
        case 1:
            *dx = 1;
            *dy = 1;
            break;
        case 2:
            *dx = 0;
            *dy = 1;
            break;
        case 3:
            *dx = -1;
            *dy = 1;
            break;
        case 4:
            *dx = -1;
            *dy = 0;
            break;
        case 5:
            *dx = -1;
            *dy = -1;
            break;
        case 6:
            *dx = 0;
            *dy = -1;
            break;
        case 7:
            *dx = 1;
            *dy = -1;
            break;
        default:
            ROS_ERROR("Invalid direction %d!", *dir);
            return false;
    }
    
    return true;
}

/**
 * Send a goal to the agent's move base node.
 * It moves the robot by the given parameters,
 * using an action client.
 * @param int vel_hor: horizontal velocity of agent.
 * @param int vel_ver: vertical velocity of agent.
 * @return bool: success of movement.
 */
bool move(int vel_hor, int vel_ver)
{
    actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction> ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0)))
        ROS_DEBUG_ONCE("Waiting for action server.");

    move_base_msgs::MoveBaseGoal goal_msgs;

    goal_msgs.target_pose.header.seq += 1;
    goal_msgs.target_pose.header.stamp = ros::Time::now();
    goal_msgs.target_pose.header.frame_id = "map";
    goal_msgs.target_pose.pose.position.x = round(pose.position.x) + vel_hor * step_size;
    goal_msgs.target_pose.pose.position.y = round(pose.position.y) + vel_ver * step_size;
    goal_msgs.target_pose.pose.position.z = 0;
    goal_msgs.target_pose.pose.orientation = pose.orientation;
    //goal_msgs.target_pose.pose.orientation.x = 0;
    //goal_msgs.target_pose.pose.orientation.y = 0;
    //goal_msgs.target_pose.pose.orientation.z = 0;
    //goal_msgs.target_pose.pose.orientation.w = 1;
    
    // quaternion: rotation of theta around vector (x,y,z): <cos(theta/2), x*sin(theta/2), y*sin(theta/2), z*sin(theta/2)>
    
    // see tf::Quaternion(x,y,z,w);
    // .getAngle()
    
    ROS_DEBUG("Move agent to (%.2f,%.2f).", goal_msgs.target_pose.pose.position.x, goal_msgs.target_pose.pose.position.y);

    ac.sendGoal(goal_msgs);
    ac.waitForResult(ros::Duration(7.0));

    while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        ROS_DEBUG_ONCE("Action client is PENDING.");

    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        ROS_DEBUG_ONCE("Action client is ACTIVE.");

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Action client ABORTED!");
            return false;
            break;
        }
    }

    ROS_DEBUG("Reached goal (%.2f,%.2f).", goal_msgs.target_pose.pose.position.x, goal_msgs.target_pose.pose.position.y);
    return true;
}

/**
 * Move the agent by into a given direction by using the move(double,double) function.
 * @param int direction: The direction to move the agent to.
 *                       3 2 1
 *                       4   0
 *                       5 6 7
 * @return bool: success of movement.
 */
 bool move(int dir)
 {
    int dx,dy;
    dir2coord(&dir, &dx, &dy);
    return move(dx, dy);
 }

/**
 * Callback function to receive agent position.
 * @param const geometry_msgs::PoseWithCovarianceStamped newPose: the agents position.
 */
void poseUpdate(const geometry_msgs::PoseWithCovarianceStamped new_pose)
{
    pose = new_pose.pose.pose;
    pose_valid = true;
    ROS_DEBUG("Got valid position, agent at (%.2f,%.2f).", pose.position.x, pose.position.y);
}

void mapUpdate(const nav_msgs::OccupancyGrid new_grid_map)
{
    grid_map = new_grid_map;
    map_valid = true;
    ROS_DEBUG("Got valid map of size %.2f x %.2f.", grid_map.info.width, grid_map.info.height);
}

/**
 * Calculate the eucledian distance between two coordinates.
 * @param pair<double,double> c1: first coordinate.
 * @param pair<double,double> c2: second coordinate.
 * @return double: the eucledian distance between the coordinates.
 */
double distance(pair<double,double> c1, pair<double,double> c2)
{
    return hypot(c2.first-c1.first, c2.second-c1.second);
}

/**
 * Initialize the log file.
 */
void initLog()
{
    //std::stringstream robot_number;
    //robot_number << robot_id;
    //std::string prefix = "/robot_";
    //std::string robo_name = prefix.append(robot_number.str());

    //log_path = log_path.append("/explorer");
    //log_path = log_path.append(robo_name);

    // create log directory
    //boost::filesystem::path boost_log_path(log_path.c_str());
    //if(!boost::filesystem::exists(boost_log_path))
    //    try{
    //        if(!boost::filesystem::create_directories(boost_log_path))
    //            ROS_ERROR("Cannot create directory %s.", log_path.c_str());
    //    }catch(const boost::filesystem::filesystem_error& e)
    //    {
    //        ROS_ERROR("Cannot create path %s.", log_path.c_str());
    //    }

    //log_file = log_file.append("/");
    
    // log file path
    log_file = log_file.append("test.log"); // TODO
    log_valid = true;
    ROS_INFO("Logging to %s", log_file.c_str());
    
    // write log file header
    fs_log.open(log_file.c_str(), fstream::trunc | fstream::out);
    fs_log << "# emergency exit example" << endl;
    fs_log << "#" << endl << endl;
    fs_log << "# step" << "\t" << "dist" << endl;
    fs_log.close();
}

/**
 * Write current status to log file.
 * @return bool: Returns false if the log file has not been initialized yet.
 */
bool log()
{
    if(log_valid == false){
        ROS_ERROR("No valid log file!");
        return false;
    }
    
    fs_log.open(log_file.c_str(), fstream::app | fstream::out);
    fs_log << step << "\t" << closest_dist << endl;
    fs_log.close();
    
    return true;
}

/**
 * Read the map data around the agent.
 * @param int direction: Which of the neighboring cells to read.
 *                       3 2 1
 *                       4   0
 *                       5 6 7
 * @return int: occupancy probability of cell, -1=unknown, 0=free, 100=occupied.
 */
int sense(int direction)
{
    // convert direction to dx and dy
    int dx,dy;
    if(dir2coord(&direction, &dx, &dy) == false){
        ROS_ERROR("Invalid direction %d!", direction);
        return -1;
    }
    
    // compute index
    int x = round(pose.position.x) + dx; // add offset
    int y = round(pose.position.y) + dy;    
    x /= grid_map.info.resolution; // convert meters to pixels
    y /= grid_map.info.resolution;
    x += grid_map.info.width / 2; // shift center to bottom left
    y += grid_map.info.height / 2;
    x += step_size * grid_map.info.resolution / 2; // shift to center of square meter
    y += step_size * grid_map.info.resolution / 2;
    int idx = y * grid_map.info.width + x;
    ROS_DEBUG("%d = %d * %d + %d", idx, y, grid_map.info.width, x);
    ROS_DEBUG("Map data at (%d,%d), idx %d: %d.", x, y, idx, grid_map.data.at(idx));
    
    // check if valid index
    if(idx < 0 || grid_map.data.size() <= idx){
        ROS_ERROR("Invalid map index %d!", idx);
        return -1;
    }
    
    // read map data
    return grid_map.data.at(idx);
}

/**
 * Main function, entrypoint.
 * @param int arc: number of passed arguments.
 * @param char** argv: the passed arguments.
 * @return int: exit code of the program.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");

    ros::NodeHandle nh("emergency_exit");
    
    // set logger level
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    //    ros::console::notifyLoggerLevelsChanged();
    //}
    //else{
    //    ROS_ERROR("Could not set logger level!");
    //}
    
    // subscribe to /amcl_pose to get agent position
    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 1, poseUpdate);
    
    // subscribe to /map to get grid map of environment
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapUpdate);
        
    // neural network controller
    //cv::Ptr< cv::ml::ANN_MLP > nn = cv::ml::ANN_MLP::create();
    
    // read nn configuration
    // TODO
    
    // setup nn as done in frevo
    // TODO
    
    // initialize log file
    nh.param<string>("log_path", log_file, "");
    initLog();
    
    // read maximum number of simuation steps
    int max_steps;
    nh.param<int>("steps", max_steps, 100);
    step = 0;
    
    // read coordinates of emergency exits
    int num_exits;
    nh.param<int>("exits", num_exits, 0);
    ROS_DEBUG("There are %d emergency exits.", num_exits);
    list<pair<double,double>> exits;
    list<pair<double,double>>::iterator it;
    for(int i=0; i<num_exits; ++i){
        stringstream exit_s;
        exit_s << "exit_" << i;
        vector<double> exit_i;
        nh.getParam(exit_s.str(), exit_i);
        ROS_DEBUG("Exit %d at (%.2f,%.2f).", i, exit_i[0], exit_i[1]);
        exits.push_back(make_pair(exit_i[0], exit_i[1]));
    }
    if(exits.size() == 0){
        ROS_FATAL("There are no emergency exits, terminating now!");
        return 0;
    }
    
    // read escape route
    vector<int> escape_route;
    nh.getParam("escape_route", escape_route);
    if(escape_route.size() > 0){
        stringstream ss_er;
        for(vector<int>::iterator it = escape_route.begin(); it != escape_route.end(); ++it)
            ss_er << *it;
        ROS_DEBUG("Escape route: %s", ss_er.str().c_str());
    }
    
    // read epsilon which is the allowed distance between two points
    // to be still considered at the same location
    double epsilon;
    nh.param<double>("epsilon", epsilon, 0.1);
    
    // read step size which is the distance the agent travels in each simulation step
    nh.param<double>("step_size", step_size, 1);
    
    // sensor inputs
    //  -1: unknown
    //   0: free
    // 100: occupied
    int in[8];
    
    // true when agent found an exit
    bool escaped = false;
        
    // frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);
    
    // wait to get a valid agent position
    while(pose_valid == false){
        ROS_DEBUG_ONCE("Waiting for valid pose.");
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    // wait to get a valid map
    while(map_valid == false){
        ROS_DEBUG_ONCE("Waiting for valid map.");
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    // initialize random seed
    srand(time(NULL));

    // run simulation step by step
    while(ros::ok() && escaped == false){
        ROS_DEBUG("Simulating step %d.", step);
        // get updates from subscriptions
        ros::spinOnce();
        
        // sense neighborhood
        for(int i=0; i<8; ++i){
            in[i] = sense(i);
        }
        
        // find closest exit
        closest_dist = 0;
        pair<double,double> closest_exit;
        for(it=exits.begin(); it!=exits.end(); ++it){
            double dist = distance(*it, make_pair(pose.position.x, pose.position.y));
            if(dist < closest_dist || closest_dist == 0){
                closest_dist = dist;
                closest_exit = *it;
            }
        }
        ROS_DEBUG("Closest emergency exit at (%.2f,%.2f), distance %.2f.", closest_exit.first, closest_exit.second, closest_dist);

        // agent reached exit
        if(closest_dist < epsilon){
            escaped = true;
            ROS_INFO("Agent escaped.");
            break;
        }
        
        // maximum number of steps reached
        if(step >= max_steps)
            break;
        
        // agent continues to search for exit
        else{
            // use nn to get actuator output
            // TODO
            //double vel_hor = 0;
            //double vel_ver = 1;
            //ROS_DEBUG("Move agent by (%.2f,%.2f).", vel_hor, vel_ver);
            int dir = rand() % 8;
            if(step < escape_route.size())
                dir = escape_route[step];
            else{
                while(in[dir] > 0)
                    dir = rand() % 8;
            }
            ROS_INFO("Step %d: move agent to %d.", step, dir);
            
            // move agent
            //if(move(vel_hor, vel_ver) == false){
            //    ROS_ERROR("Could not move agent to (%.2f, %.2f)!", pose.position.x+vel_hor, pose.position.y+vel_ver);
            //}
            if(move(dir) == false){
                ROS_ERROR("Could not move agent to %d!", dir);
            }
        }
        
        // write current status to log file
        log();
        
        // sleep for 1/rate seconds
        loop_rate.sleep();
        
        // next step
        ++step;
    }
        
    // write current status to log file
    log();

    return 0;
}

