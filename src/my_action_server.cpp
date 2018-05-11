// example_action_server: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<my_action_server/demoAction.h>
#include <std_msgs/Bool.h> 
#include <my_action_server/demoAction.h>
#include <math.h>
#include <ros/ros.h>

#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


int g_count = 0;
bool g_count_failure = false;

const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
double robot_heading; //robot's actual heading (not 0!!!)

geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);




class MyActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<my_action_server::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_action_server::demoGoal goal_; // goal message, received from client
    my_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::demoFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    MyActionServer(); //define the body of the constructor outside of class definition

    ~MyActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_action_server::demoAction>::GoalConstPtr& goal);
};

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        while (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        while (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}


void do_inits(ros::NodeHandle &nh_) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
     
}

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

MyActionServer::MyActionServer() :
   as_(nh_, "path_action", boost::bind(&MyActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running

    

}



//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void MyActionServer::executeCB(const actionlib::SimpleActionServer<my_action_server::demoAction>::GoalConstPtr& goal) {
    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    nav_msgs::Path paths;
    geometry_msgs::Pose pose_next;
    geometry_msgs::Pose orientation_next;
    paths = goal->paths;
    int num_points = paths.poses.size();
   


    for(int i = 0;num_points;i++){

        pose_next = paths.poses[i].pose;
     
        double x = g_current_pose.position.x;
        double y = g_current_pose.position.y;
        double x_ = pose_next.position.x;
        double y_ = pose_next.position.y;
        double dy = y_ - y;
        double dx = x_ - x;
        double travel_dist = dx*dx + dy*dy;
         travel_dist = sqrt(travel_dist);
        double heading = atan2(dy,dx);
        do_spin(heading);//spins to see next waypoint.
        //check for canceled goal
        if(as_.isPreemptRequested())
        {
          do_halt();    
          ROS_WARN("goal cancelled!");
          result_.output = -1;
          as_.setAborted(result_); // 
          return; // done with callback
        }
        //if not canceled via lidar start moving
        do_move(travel_dist);
        //then reorient to desired orientation 
        heading = heading -convertPlanarQuat2Phi(pose_next.orientation);

        do_spin(heading);
        g_current_pose = pose_next; //update positon and heading.





    }
    //....

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    g_count++; // keep track of total number of goals serviced since this server was started
    
    result_.goal_stamp = goal->input;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date

    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");
    ros::NodeHandle nh_;
  

    do_inits(nh_);


    MyActionServer as_object; // create an instance of the class "ExampleActionServer"

    g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (!g_count_failure && ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}
