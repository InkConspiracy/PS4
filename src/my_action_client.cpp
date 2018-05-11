// example_action_client: 
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../example_action_server/action/demo.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (demo) and appended name (Action)
// If you write a new client of the server in this package, you will need to include example_action_server in your package.xml,
// and include the header file below
#include <std_msgs/Bool.h> 
#include <my_action_server/demoAction.h>
#include <math.h>
#include <ros/ros.h>

#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
bool g_galarm_trigger = false; 
bool g_lidar_alarm=false; // global var for lidar alarm
ros::Publisher g_twist_commander;
//demo points for adding to the whole shebang.

void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
  }
} 

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}



void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d", result->output, result->goal_stamp, diff);
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "example_action_client_node"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(1.0);
        ros::init(argc, argv, "commander"); 
        ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
        ros::Subscriber lidar_alarm_sub = n.subscribe("lidar_alarm", 1, alarmCallback);
      
        geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
        // start with all zeros in the command message; should be the case by default, but just to be safe..
        twist_cmd.linear.x=0.0;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;   
  
    double timer=0.0;
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds
    
    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate   
    
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        my_action_server::demoGoal goal; 
        geometry_msgs::Quaternion quat;
        
        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<my_action_server::demoAction> action_client("path_action", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        
        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        std::vector<geometry_msgs::PoseStamped> plan;
        nav_msgs::Path paths;
        
        //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      ros::spinOnce();
      loop_timer.sleep();
    }
        
        while(ros::ok()) {
      
      twist_cmd.angular.z=0.0; // do not spin 
        //twist_cmd.linear.x=speed; //command to move forward
       
    while(!g_lidar_alarm){
      twist_commander.publish(twist_cmd);
          timer+=sample_dt;
      
           plan.clear();
           
           pose.position.x = 3.0; // say desired x-coord is 3
           pose.position.y = 0.0;
           pose.position.z = 0.0; // let's hope so!
           pose.orientation.x = 0.0; //always, for motion in horizontal plane
           pose.orientation.y = 0.0; // ditto
           pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
           pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);


           pose.position.x = 3; // say desired x-coord is 3
           pose.position.y = 3;
           pose.orientation = convertPlanarPhi2Quaternion(3.14/2);
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
          pose.position.x = 7; // say desired x-coord is 3
           pose.position.y = 3;
           pose.orientation = convertPlanarPhi2Quaternion(0);
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           pose.position.x = 7; // say desired x-coord is 3
           pose.position.y = 8;
           pose.orientation = convertPlanarPhi2Quaternion(3.14/2);
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);





           
           paths.poses = plan;
           goal.paths = paths;
           action_client.sendGoal(goal, &doneCb);
           ros::spinOnce();
           loop_timer.sleep();
     }
     
     action_client.cancelGoal();
     plan.clear();
     
     //Alarm
        twist_cmd.linear.x=0.0; //stop moving forward
        twist_cmd.angular.z=yaw_rate; //and start spinning left
        
     
     pose.position.x = 0.0; 
       pose.position.y = 0.0;
       pose.position.z = 0.0; 
       pose_stamped.pose = pose;
       plan.push_back(pose_stamped);
           

           
       paths.poses = plan;
       ROS_INFO("Stopping!");
       goal.paths = paths;
       timer=0.0; //reset the timer
     
     while(!g_lidar_alarm){
       
       plan.clear();
       twist_commander.publish(twist_cmd);
           timer+=sample_dt;
         
      
       pose.position.x = 0.0;
       pose.position.y = 0.0;
       pose.position.z = 0.0;
       pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           paths.poses = plan;
           goal.paths = paths;
           action_client.sendGoal(goal, &doneCb);
           
           ros::spinOnce();
           loop_timer.sleep();
       
     }
           
       }
    return 0;
}


