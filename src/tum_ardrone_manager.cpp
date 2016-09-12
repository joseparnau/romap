/*
  Software License Agreement (BSD License)

  Copyright (c) 2015, Josep-Arnau Claret Robert
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  author: Josep Arnau Claret Robert
*/

#include <sstream>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <std_srvs/Empty.h>

#include <romap/SetTrackTcpOn.h>

#include <ardrone_autonomy/Navdata.h>
#include <uga_tum_ardrone/filter_state.h>

#ifndef PI
#define PI 3.141592
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(x) x*(PI/180)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(x) x*(180/PI)
#endif

// States
#define STATE_REST   0
#define STATE_CALIB_SEARCH 1
#define STATE_TRACK  2

// Ardrone States
#define ARDRONE_OFF       0
#define ARDRONE_INIT      1
#define ARDRONE_LANDED    2
#define ARDRONE_TAKINGOFF 7
#define ARDRONE_FLYING_1  3
#define ARDRONE_FLYING_2  4
#define ARDRONE_LANDING   9

// State transitions ----------------------------------------------

// STATE_REST   + (track_tcp)                      --> STATE_CALIB_SEARCH

// STATE_CALIB_SEARCH + (track_tcp, tcp detected)  --> STATE_TRACK
// STATE_CALIB_SEARCH + (!track_tcp)               --> STATE_REST

// STATE_TRACK  + (track_tcp, !tcp detected)       --> STATE_CALIB_SEARCH
// STATE_TRACK  + (!track_tcp)                     --> STATE_REST

// ----------------------------------------------------------------

// Initial state
static unsigned int state = STATE_REST;
static bool track_tcp = false;
static bool tcp_detected = false;
static bool flattrim_calibrated = false;
static unsigned int ardrone_state = ARDRONE_OFF;

// Tcp search variables
//  Ardrone field of view is 60º
#define SEARCH_ANGLE_BLOCK 45	// º
#define SEARCH_HEIGTH_MIN 0.7	// m
#define SEARCH_HEIGTH_MAX 1.7	// m

// Security information
#define SECURITY_FLOOR_SQUARE_SIDE 1.6//1.6 // m
#define SECURITY_MIN_HEIGTH        0.1 // m
#define SECURITY_MAX_HEIGTH        1.7 // m


// ROS Subscribers
void ardroneAutonomyLandCallback(const std_msgs::Empty::ConstPtr& msg)
{
  track_tcp = false;
  return;
}

void detectedTCPCallback(const std_msgs::Bool::ConstPtr& msg)
{
  tcp_detected = msg->data;
  return;
}

void ardroneNavDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
  ardrone_state = msg->state;
//   ROS_INFO("ar state: %d", ardrone_state);
  return;
}

static nav_msgs::Odometry ardroneOdometryPoseTwist;
void ardroneOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ardroneOdometryPoseTwist = *msg;
  return;
}

bool newDesiredPoseReceived = false;
bool firstDesiredTcpToArdronePose_received = false;
static geometry_msgs::PoseStamped desiredTcpToArdronePose, pastDesiredTcpToArdronePose;
tf::StampedTransform desired_TF_tcp_cam, pastDesired_TF_tcp_cam;
void desiredArdroneTcp2CamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  desiredTcpToArdronePose = *msg;
  
  if (firstDesiredTcpToArdronePose_received){
    pastDesiredTcpToArdronePose.pose = desiredTcpToArdronePose.pose;
    pastDesiredTcpToArdronePose.header.stamp.sec = desiredTcpToArdronePose.header.stamp.sec-1;
    firstDesiredTcpToArdronePose_received = true;
  }
  
  desired_TF_tcp_cam.setData(tf::Transform(tf::Quaternion(desiredTcpToArdronePose.pose.orientation.x,
							  desiredTcpToArdronePose.pose.orientation.y,
							  desiredTcpToArdronePose.pose.orientation.z,
							  desiredTcpToArdronePose.pose.orientation.w),
					   tf::Vector3(desiredTcpToArdronePose.pose.position.x,
						       desiredTcpToArdronePose.pose.position.y,
						       desiredTcpToArdronePose.pose.position.z)
					  )
			    );
  desired_TF_tcp_cam.stamp_ = msg->header.stamp;
  desired_TF_tcp_cam.frame_id_ = "/tcp";
  desired_TF_tcp_cam.child_frame_id_ = "/desired_cam";
  
  if (firstDesiredTcpToArdronePose_received){
    pastDesired_TF_tcp_cam.setData(desired_TF_tcp_cam);
    pastDesired_TF_tcp_cam.stamp_ = ros::Time(desired_TF_tcp_cam.stamp_.sec-1,desired_TF_tcp_cam.stamp_.nsec);
    firstDesiredTcpToArdronePose_received = true;
  }
    
  newDesiredPoseReceived = true;
    
  return;
}


static uga_tum_ardrone::filter_state predictedArdronePose;
void predictedArdronePoseCallback(const uga_tum_ardrone::filter_state::ConstPtr& msg)
{
  predictedArdronePose = *msg;

  return;
}


// ROS Services
bool trackTcpServiceCallback(romap::SetTrackTcpOn::Request &req, romap::SetTrackTcpOn::Response &res)
{
  track_tcp = req.track_tcp;
  if (track_tcp  &&  ardrone_state <= ARDRONE_LANDED)					ROS_INFO("Track TCP: ON");
  else if (ardrone_state > ARDRONE_LANDED  &&  ardrone_state != ARDRONE_LANDING)	ROS_INFO("Track TCP: OFF");
  return true;
}

bool move_on_track = false;
bool moveOnTrackServiceCallback(romap::SetTrackTcpOn::Request &req, romap::SetTrackTcpOn::Response &res)
{
  move_on_track = req.track_tcp;
  ROS_INFO("Move on Track: ON");
  return true;
}

// Functions

double Zangle_uXDrone_uZTCP(const tf::Vector3 vector);

tf::Matrix3x3 cross_product_matrix_S(const tf::Vector3 vector);

int sgn(double val){	return (0.0 < val) - (val < 0.0);	}

Eigen::VectorXd computeDroneVelocityCommand(const double t_time_diff, const tf::Transform t_TF_drone_tcp, const tf::Transform t_pastTF_drone_tcp, 
					    const tf::Transform t_desired_TF_drone_tcp, const tf::Transform t_pastDesired_TF_drone_tcp);


// Filter
#define F_ORDER_ZEROS 1
#define F_ORDER_POLES 1
unsigned int n_samples = 0;
double a[F_ORDER_ZEROS+1], b[F_ORDER_POLES+1];
double xv[F_ORDER_POLES+1][4]; // Input
double yv[F_ORDER_ZEROS+1][4]; // Output
Eigen::VectorXd signalFilterOutput(const Eigen::VectorXd src, const unsigned int dim);
void loadFilterValues(double a[], const unsigned int aDim, double b[], const unsigned int bDim);


int main(int argc, char *argv[])
{
  // ROS
  ros::init (argc, argv, "ardrone_manager");
  ros::NodeHandle n;
  
  // Publishers
  ros::Publisher pubTcpTracking = n.advertise<std_msgs::Bool>("tcp_tracking", 1);
//   ros::Publisher pubArdroneTakeOff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
//   ros::Publisher pubArdroneLand = n.advertise<std_msgs::Empty>("ardrone/land", 1);
//   ros::Publisher pubArdroneTwist = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher pubTUMArdroneCommand = n.advertise<std_msgs::String>("uga_tum_ardrone/com", 1);  
  
  // Subscribers
  ros::Subscriber subArdroneAutonomyLand = n.subscribe("ardrone/land", 1, ardroneAutonomyLandCallback);  
  ros::Subscriber subTcpDetected = n.subscribe("tcp_detected", 1, detectedTCPCallback);
  ros::Subscriber subArdroneNavData = n.subscribe("ardrone/navdata", 1, ardroneNavDataCallback);
  ros::Subscriber subArdroneOdometry = n.subscribe("ardrone/odometry", 1, ardroneOdometryCallback);
  ros::Subscriber subDesiredArdronePose = n.subscribe("ardrone_tcp_cam_desired_pose", 1, desiredArdroneTcp2CamPoseCallback);  
  ros::Subscriber subPredictedArdronePose = n.subscribe("ardrone/predictedPose", 1, predictedArdronePoseCallback);  
  
  
  // Service Clients
  ros::ServiceClient clientArdroneFlatTrimCalibration = n.serviceClient<std_srvs::Empty>("ardrone/flattrim");
  
  // Service Servers
  ros::ServiceServer serviceTrackTcpServiceCallback = n.advertiseService("track_tcp", trackTcpServiceCallback);
  ros::ServiceServer serviceMoveOnTrackServiceCallback = n.advertiseService("move_on_track", moveOnTrackServiceCallback);  
  
  std::ofstream outFile("/home/users/josep.a.claret/SparkleShare/iocdocs/projectes/romap/experiments/lwr_impedance/time_ardrone_dsd_vel.dat");

  // Wait for TFs
  double wait_time = 60.0;	// seconds
  tf::TransformListener TFListener;
  TFListener.waitForTransform("/tcp", "/camera", ros::Time(0), ros::Duration(wait_time));
  TFListener.waitForTransform("/odom", "/ardrone_base_link", ros::Time(0), ros::Duration(wait_time) );
  TFListener.waitForTransform("/ardrone_base_link", "/ardrone_base_frontcam", ros::Time(0), ros::Duration(wait_time) ); 

  
  // Initialize
  desired_TF_tcp_cam.setData(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0) ) );    
  
  tf::StampedTransform ardroneSearchPose;
  
  bool cube_just_detected = false;
  geometry_msgs::PoseStamped currentTcpToArdronePose, pastTcpToArdronePose, firstTcpToArdronePose;  
  currentTcpToArdronePose.pose.position.x = currentTcpToArdronePose.pose.position.y = currentTcpToArdronePose.pose.position.z = 0.0;
  currentTcpToArdronePose.pose.orientation.x = currentTcpToArdronePose.pose.orientation.y = currentTcpToArdronePose.pose.orientation.z = 0.0;
  currentTcpToArdronePose.pose.orientation.w = 1.0;
  pastTcpToArdronePose = firstTcpToArdronePose = currentTcpToArdronePose;
  
  pastDesiredTcpToArdronePose = desiredTcpToArdronePose = currentTcpToArdronePose;
  
  bool security_on = true;
  if ( atoi(argv[1]) == 0 )	security_on = false;
  
  double rest_timer, search_timer, track_timer, move_on_track_timer;
  rest_timer = search_timer = track_timer = move_on_track_timer = 0.0;

  // UGA drone variables
  bool PTAMmapLocked = false;
  unsigned int c_uga_start = 0;  
  unsigned int c_uga_takeoff = 0;
  
  int freq = 50;
  double period = 1.0/static_cast<double>(freq);
  ros::Rate loop_rate(freq);
  double curr_time = ros::Time::now().toSec();  
  while (ros::ok())
  {
    double past_time = curr_time;
    curr_time = ros::Time::now().toSec();
    double period_real = curr_time - past_time;

    double time_diff = period;
    
    // Initialize
    //  Default command twist is zero
    bool ardrone_commanded = false;
    geometry_msgs::Twist commandTwist;    
    commandTwist.linear.x = commandTwist.linear.y = commandTwist.linear.z = 0.0;
    commandTwist.angular.x = commandTwist.angular.y = commandTwist.angular.z = 0.0;

    // TF
    tf::StampedTransform TF_world_ardrone;
    TFListener.lookupTransform("/odom", "/ardrone_base_link", ros::Time(0), TF_world_ardrone);

    // Input variables
    tf::Vector3 Ardrone_InputCommand_linearVelocity, Ardrone_InputCommand_angularVelocity;
    Ardrone_InputCommand_linearVelocity = tf::Vector3(0.0, 0.0, 0.0);
    Ardrone_InputCommand_angularVelocity = tf::Vector3(0.0, 0.0, 0.0);
    
    tf::StampedTransform TF_tcp_cam, pastTF_tcp_cam, firstTF_tcp_cam;
    if (!cube_just_detected)	pastTF_tcp_cam = TF_tcp_cam;		// Store past pose	  
    TFListener.lookupTransform("/tcp", "/camera", ros::Time(0), TF_tcp_cam);
    if (cube_just_detected){	// if there is no past pose then: past pose = current pose
      firstTF_tcp_cam = pastTF_tcp_cam = TF_tcp_cam;
      pastTF_tcp_cam.stamp_ = ros::Time(TF_tcp_cam.stamp_.sec-1,TF_tcp_cam.stamp_.nsec);
      cube_just_detected = false;
    }

    //  Desired TCP to Cam Pose
//       desired_TF_tcp_cam.setData(tf::Transform(tf::Quaternion(1.0, 0.0, 0.0, 0.0), tf::Vector3(0.0, 0.0, 2.0)));	// TEST TEST TEST
    if (!newDesiredPoseReceived)	pastDesired_TF_tcp_cam = desired_TF_tcp_cam;
    
    //  Poses: drone to TCP
    tf::StampedTransform TF_drone_cam;
    TFListener.lookupTransform("/ardrone_base_link", "/ardrone_base_frontcam", ros::Time(0), TF_drone_cam);    
    //   Current
    tf::Transform TF_drone_tcp = TF_drone_cam * TF_tcp_cam.inverse();
    tf::Transform pastTF_drone_tcp = TF_drone_cam * pastTF_tcp_cam.inverse();
    //   Desired
    tf::Transform desired_TF_drone_tcp = TF_drone_cam * desired_TF_tcp_cam.inverse();
    tf::Transform pastDesired_TF_drone_tcp = TF_drone_cam * pastDesired_TF_tcp_cam.inverse();
    
    // State machine
    switch (state)
    {
      // Ardrone resting at the floor -----------------------------------------------------------------------------
      case STATE_REST:
      {
	rest_timer += period_real;
	
	if ( ardrone_state == ARDRONE_LANDED ){
	  if ( !flattrim_calibrated )
	  {
	    std_srvs::Empty srv;
	    clientArdroneFlatTrimCalibration.call(srv);
	    ros::spinOnce();
	    flattrim_calibrated = true;
	    ROS_INFO ("Flattrim Calibration DONE");
	  }
	  
	  if (!c_uga_start)
	  {
	    std_msgs::String TumAutopilotCommand;
	    TumAutopilotCommand.data = "c clearCommands";
	    pubTUMArdroneCommand.publish(TumAutopilotCommand);
	    ros::spinOnce();
	    
	    TumAutopilotCommand.data = "c start";
	    pubTUMArdroneCommand.publish(TumAutopilotCommand);	    
	    
	    ++c_uga_start;

	    ROS_INFO("Starting UGA TUM ardrone ...");
	  }
	}
	else{
	  c_uga_start = c_uga_takeoff = 0;
	}
	  
	
// 	std::cout << "tcp - cam: " <<desired_TF_tcp_cam.getOrigin().x()<<" "  
// 				   <<desired_TF_tcp_cam.getOrigin().y()<<" " 
// 				   <<desired_TF_tcp_cam.getOrigin().z()<<std::endl;
	
	// To start commanding the drone first do:
	//  $ rosservice call /track_tcp "track_tcp: true"
	//  The ardrone state will change from STATE_REST to STATE_CALIB_SEARCH
	//  Once the cube is detected the state will automatically change from STATE_CALIB_SEARCH to STATE_TRACK
	//  In the state STATE_TRACK the tracking controller will automatically be started


	// TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
//	if (flattrim_calibrated && (ardrone_state == ARDRONE_LANDED) && tcp_detected)
//	{
//// 	  std::cout << "In control test!" << std::endl;
//	  
//	  Eigen::VectorXd u_drone(computeDroneVelocityCommand(time_diff, TF_drone_tcp, pastTF_drone_tcp, desired_TF_drone_tcp, pastDesired_TF_drone_tcp));
//	  double ux = u_drone(0);
//	  double uy = u_drone(1);
//	  double uz = u_drone(2);
//	  double ut = u_drone(3);	  
//	  
//	  Ardrone_InputCommand_linearVelocity = tf::Vector3(ux, uy, uz);
//	  Ardrone_InputCommand_angularVelocity = tf::Vector3(0.0, 0.0, ut);
//
//	  ardrone_commanded = true;
// 	  commandTwist.angular.x = commandTwist.angular.y = 0.1;	// Do not allow to enter auto-hover mode
//
//	  Ardrone_InputCommand_linearVelocity = tf::Vector3(0.0, 0.0, 0.0);
//	  Ardrone_InputCommand_angularVelocity = tf::Vector3(0.0, 0.0, 0.0);
//	  
// 	  
//// 	  outFile << rest_timer<<" "<<ux<<" "<<uy<<" "<<uz<<" "<<ut<<" "<<std::endl;
//	}
	// TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

	

	// Ardrone is flying: Land it
// 	if ( ardrone_state > ARDRONE_LANDED  &&  ardrone_state != ARDRONE_LANDING)
// 	{
// 	  commandTwist.linear.x = commandTwist.linear.y = commandTwist.linear.z = 0.0;
// 	  commandTwist.angular.x = commandTwist.angular.y = commandTwist.angular.z = 0.0;
// 	  pubArdroneTwist.publish(commandTwist);
// 	  
// 	  ros::Duration(1.0).sleep();	// Sleep to let ardrone to stop before landing
//   
// 	  std_msgs::Empty emptyMsg;
// 	  pubArdroneLand.publish(emptyMsg);
// 	  ROS_INFO("Landing ...");
// 
// 	  ardrone_commanded_to_land_but_not_landed = true;
// 	}

	
	//   State machine transitions
	if (    flattrim_calibrated  
	     && track_tcp  
	     && c_uga_start 
	     && ( ardrone_state == ARDRONE_LANDED )  
	     && (    !security_on 
		  || (    security_on 
		       && fabs(TF_world_ardrone.getOrigin().x()) < SECURITY_FLOOR_SQUARE_SIDE/2.0 
		       && fabs(TF_world_ardrone.getOrigin().y()) < SECURITY_FLOOR_SQUARE_SIDE/2.0 
		     ) 
		) 
	   )
	{
	  state = STATE_CALIB_SEARCH;
	  ardroneSearchPose = TF_world_ardrone;
	  rest_timer = search_timer = track_timer = 0.0;
	}
	
	break;
      }
	
	
      // Ardrone searching for the TCP -----------------------------------------------------------------------------
      case STATE_CALIB_SEARCH:
      {
	search_timer += period_real;
	
	ardrone_commanded = true;
	commandTwist.linear.x  = commandTwist.linear.y  = commandTwist.linear.z  = 0.0;
	commandTwist.angular.x = commandTwist.angular.y = commandTwist.angular.z = 0.0;
	
	if (ardrone_state == ARDRONE_LANDED){	// Ardrone is not flying: Take off
	  
	  if (c_uga_takeoff == 0){
  // 	  std_msgs::Empty emptyMsg;
  // 	  pubArdroneTakeOff.publish(emptyMsg);
	    
	    std_msgs::String TumAutopilotCommand;
// 	    TumAutopilotCommand.data = "c autoInit 500 800 4000 0.5";	    
	    TumAutopilotCommand.data = "c takeoff";
	    pubTUMArdroneCommand.publish(TumAutopilotCommand);
	    
// 	    ROS_INFO("predicted pose:  %f %f %f | %f", predictedArdronePose.x, predictedArdronePose.y, predictedArdronePose.z, 
// 						       predictedArdronePose.yaw);
	    
	    if (!PTAMmapLocked){
	      ros::spinOnce();
	      TumAutopilotCommand.data = "p toggleLockMap";
	      pubTUMArdroneCommand.publish(TumAutopilotCommand);
	      PTAMmapLocked = true;
	    }
	    
// 	    TumAutopilotCommand.data = "p moveByRel 0.0 0.0 0.0 0.0";
// 	    pubTUMArdroneCommand.publish(TumAutopilotCommand);
// 	    ROS_INFO("MOVE");
	    
	    ROS_INFO("Taking off and searching TCP ...");
	    
	    ++c_uga_takeoff;
	  }
	}
	
	//   State machine transitions
	if (track_tcp)
	{
	  if (     tcp_detected  
	       &&  (    (ardrone_state == ARDRONE_FLYING_1) 
	             || (ardrone_state == ARDRONE_FLYING_2)
	           )
	  ){		 
	    state = STATE_TRACK;
	    cube_just_detected = true;
	    ROS_INFO("Tcp detected");
	    rest_timer = search_timer = track_timer = 0.0;
	  }

	  if ( security_on &&
	       ( (fabs(TF_world_ardrone.getOrigin().x()) > SECURITY_FLOOR_SQUARE_SIDE/2.0) || 
	         (fabs(TF_world_ardrone.getOrigin().y()) > SECURITY_FLOOR_SQUARE_SIDE/2.0) || 
	         (fabs(TF_world_ardrone.getOrigin().z()) > SECURITY_MAX_HEIGTH)
	       )
	     )
	  {
	    state = STATE_REST;
	    ROS_INFO("Out of boundaries.");
	    rest_timer = search_timer = track_timer = 0.0;
	  }
	}
	else{
	  state = STATE_REST;
	  rest_timer = search_timer = track_timer = 0.0;
	}
	
	break;
      }
	
	
      // Ardrone tracking the TCP ----------------------------------------------------------------------------------
      case STATE_TRACK:
      {
	track_timer += period_real;
	
	try
	{
	  
// 	  // TCP tracking **********************************************************************
// 	  
// 	  Eigen::VectorXd u_drone(computeDroneVelocityCommand(time_diff, TF_drone_tcp, pastTF_drone_tcp, desired_TF_drone_tcp, pastDesired_TF_drone_tcp));
// 	  double ux = u_drone(0);
// 	  double uy = u_drone(1);
// 	  double uz = u_drone(2);
// 	  double ut = u_drone(3);
// 
// 	  Ardrone_InputCommand_linearVelocity = tf::Vector3(ux, uy, uz);
// 	  Ardrone_InputCommand_angularVelocity = tf::Vector3(0.0, 0.0, ut);
// 
// 	  ardrone_commanded = true;
// 	  commandTwist.angular.x = commandTwist.angular.y = 0.1;	// Do not allow to enter auto-hover mode
// 
// 	  Ardrone_InputCommand_linearVelocity = tf::Vector3(0.0, 0.0, 0.0);
// 	  Ardrone_InputCommand_angularVelocity = tf::Vector3(0.0, 0.0, 0.0);
// 	  
// 	  outFile << track_timer<<" - "<<ux<<" "<<uy<<" "<<uz<<" | "<<RAD_TO_DEG(ut)<<" "<<std::endl;
// 	  
// // 	  // PLOT
// // // 	  tf::StampedTransform TF_prov;
// // // 	  TFListener.lookupTransform("/camera", "/ardrone_base_frontcam", ros::Time(0), TF_prov);
// // 	  
// // // // 	  ROS_INFO("- TRACK ------------------------------------------");
// // // // //  	  ROS_INFO("time nT T: %f %f %f", ros::Time::now().toSec(), period, time_diff);
// // // // 	  ROS_INFO("DrnTCP P : %f %f %f", TF_drone_tcp.getOrigin().x(), TF_drone_tcp.getOrigin().y(), TF_drone_tcp.getOrigin().z());
// // // // 	  ROS_INFO("DesDrnTCP: %f %f %f", desired_TF_drone_tcp.getOrigin().x(), desired_TF_drone_tcp.getOrigin().y(), desired_TF_drone_tcp.getOrigin().z());	  
// // // // 	  ROS_INFO("x y z t  : %f %f %f %f", x, y, z, t*(180/PI));
// // // // 	  ROS_INFO("r-x y z t: %f %f %f %f", rx, ry, rz, rt*(180/PI));
// // // // 	  ROS_INFO("u-x y z t: %f %f %f %f", ux, uy, uz, ut*(180/PI));
// // //  	  ROS_INFO("tcp cam  : %f %f %f", TF_tcp_cam.getOrigin().x(), TF_tcp_cam.getOrigin().y(), TF_tcp_cam.getOrigin().z());
// // // //   	  ROS_INFO("cam tcp  : %f %f %f", TF_cam_tcp.getOrigin().x(), TF_cam_tcp.getOrigin().y(), TF_cam_tcp.getOrigin().z());
// // // // // // 	  ROS_INFO("prov     : %f %f %f", TF_prov.getOrigin().x(), TF_prov.getOrigin().y(), TF_prov.getOrigin().z());
// // // //  	  ROS_INFO("desP     : %f %f %f", desired_TF_cam_tcp.getOrigin().x(), desired_TF_cam_tcp.getOrigin().y(), desired_TF_cam_tcp.getOrigin().z());	  
// // // //  	  ROS_INFO("desTwistP: %f %f %f", desiredCamToTcp_TwistPosition.x(), desiredCamToTcp_TwistPosition.y(), desiredCamToTcp_TwistPosition.z());
// // // // // 	  ROS_INFO("desTwistO: %f %f %f", desiredCamToTcp_TwistAngular.x(), desiredCamToTcp_TwistAngular.y(), desiredCamToTcp_TwistAngular.z());	  
// // // //  	  ROS_INFO("error    : %f %f %f", Kp*CamToTcpPose_Error.getOrigin().x(), Kp*CamToTcpPose_Error.getOrigin().y(), Kp*CamToTcpPose_Error.getOrigin().z());
// // // // //  	  ROS_INFO("base2cam : %f %f %f", TF_drone_cam.getOrigin().x(), TF_drone_cam.getOrigin().y(), TF_drone_cam.getOrigin().z() );
// // // // // //  	  ROS_INFO("quat     : %f | %f %f %f", TF_drone_cam.getRotation().w(), TF_drone_cam.getRotation().x(), TF_drone_cam.getRotation().y(), TF_drone_cam.getRotation().z() );
// // // 	  ROS_INFO("mat row1 : %f %f %f", TF_drone_tcp.getBasis().getRow(0).x(), TF_drone_tcp.getBasis().getRow(0).y(), TF_drone_tcp.getBasis().getRow(0).z());
// // // 	  ROS_INFO("mat row2 : %f %f %f", TF_drone_tcp.getBasis().getRow(1).x(), TF_drone_tcp.getBasis().getRow(1).y(), TF_drone_tcp.getBasis().getRow(1).z());
// // // 	  ROS_INFO("mat row3 : %f %f %f", TF_drone_tcp.getBasis().getRow(2).x(), TF_drone_tcp.getBasis().getRow(2).y(), TF_drone_tcp.getBasis().getRow(2).z());
// // // // 	  ROS_INFO("<-in vel : %f %f %f", Ardrone_InputCommand_linearVelocity.x(), Ardrone_InputCommand_linearVelocity.y(), Ardrone_InputCommand_linearVelocity.z());
// // // // // 	  ROS_INFO("<-in vel : %f %f %f", Ardrone_InputCommand_angularVelocity.x(), Ardrone_InputCommand_angularVelocity.y(), Ardrone_InputCommand_angularVelocity.z());
// // // 	  ROS_INFO("<-inputP: %f %f %f", commandTwist.linear.x, commandTwist.linear.y, commandTwist.linear.z);
// 	  // TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST

	  
// 	  // Velocity normalizations
// 	  //  Maximum velocities
// 	  geometry_msgs::Twist maximumTwist;
// 	  maximumTwist.linear.x = 2.0;	// m/s
// 	  maximumTwist.linear.y = 2.0;	// m/s
// 	  maximumTwist.linear.z = 2.0;	// m/s
// 	  maximumTwist.angular.z = DEG_TO_RAD(180.0);	// rad/s
// 	  //  Linear velocity
// 	  if (fabs(Ardrone_InputCommand_linearVelocity.x()) > maximumTwist.linear.x)	
// 		commandTwist.linear.x = sgn(Ardrone_InputCommand_linearVelocity.x())*maximumTwist.linear.x;
// 	  else	commandTwist.linear.x = Ardrone_InputCommand_linearVelocity.x();
// 	  if (fabs(Ardrone_InputCommand_linearVelocity.y()) > maximumTwist.linear.y)
// 		commandTwist.linear.y = sgn(Ardrone_InputCommand_linearVelocity.y())*maximumTwist.linear.y;
// 	  else	commandTwist.linear.y = Ardrone_InputCommand_linearVelocity.y();
// 	  if (fabs(Ardrone_InputCommand_linearVelocity.z()) > maximumTwist.linear.z)
// 		commandTwist.linear.z = sgn(Ardrone_InputCommand_linearVelocity.z())*maximumTwist.linear.z;
// 	  else	commandTwist.linear.z = Ardrone_InputCommand_linearVelocity.z();
// 	  //  Angular velocity
// 	  if (fabs(Ardrone_InputCommand_angularVelocity.z()) > maximumTwist.angular.z)
// 		commandTwist.angular.z = sgn(Ardrone_InputCommand_angularVelocity.z())*maximumTwist.angular.z;
// 	  else	commandTwist.angular.z = Ardrone_InputCommand_angularVelocity.z();

// 	  commandTwist.linear.x *= 0.1;
// 	  commandTwist.linear.y *= 0.1;
// 	  commandTwist.linear.z *= 0.1;

// 	  commandTwist.linear.x = 0.0;
// 	  commandTwist.linear.y = 0.1*sin(2*PI*0.2*track_timer);
// 	  commandTwist.linear.z = 0.0;
	  
	  	  
// 	  ROS_INFO("<-input: %f %f %f / %f %f %f", commandTwist.linear.x, commandTwist.linear.y, commandTwist.linear.z, 
// 						   commandTwist.angular.x, commandTwist.angular.y, commandTwist.angular.z);
// 	  ROS_INFO("<-input: %f ", commandTwist.angular.z);
	  
// // 	  if (move_on_track)
// // 	  {
// // 	    move_on_track_timer += period_real;
// // // 	    ROS_INFO("in move_on_track");
// // // 	    ROS_INFO("move_on_track timer: %f", move_on_track_timer);
// // 	    ardrone_commanded = true;
// // 	    commandTwist.linear.x = 0.0;
// // 	    commandTwist.linear.y = 0.15*(1.0/2.0)*(PI/2.0)*(PI/2.0)* sin(PI - (PI/2.0)*move_on_track_timer);
// // 	    commandTwist.linear.z = 0.0;
// // 	    commandTwist.angular.x = commandTwist.angular.y = 0.0;
// // 	    commandTwist.angular.z = -0.3*(1.0/2.0)*(PI/2.0)*(PI/2.0)* sin(PI - (PI/2.0)*move_on_track_timer);;
// // 	  }
// // 	  else
// // 	  {
// // 	    move_on_track_timer = 0.0;
// // 	    ardrone_commanded = true;
// // 	    commandTwist.linear.x = 0.0;
// // 	    commandTwist.linear.y = 0.0;
// // 	    commandTwist.linear.z = 0.0;
// // 	    commandTwist.angular.x = commandTwist.angular.y = 0.0;
// // 	    commandTwist.angular.z = 0.0;
// // 	  }
	  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	//   State machine transitions
	if (track_tcp)
	{
	  // Commented so that if cube is not detected the algorithm stays in track state
// 	  if (!tcp_detected){
// 	    state = STATE_CALIB_SEARCH;
// 	    ROS_INFO("Lost track of Tcp. Searching TCP ...");
// 	    ardroneSearchPose = TF_world_ardrone;
// 	    rest_timer = search_timer = track_timer = 0.0;
// 	  }
	  
	  if ( security_on &&
	       ( (fabs(TF_world_ardrone.getOrigin().x()) > SECURITY_FLOOR_SQUARE_SIDE/2.0) || 
	         (fabs(TF_world_ardrone.getOrigin().y()) > SECURITY_FLOOR_SQUARE_SIDE/2.0) || 
	         (fabs(TF_world_ardrone.getOrigin().z()) > SECURITY_MAX_HEIGTH) 
	       ) 
	     )
	  {
	    state = STATE_REST;
	    ROS_INFO("Out of boundaries.");
	    rest_timer = search_timer = track_timer = 0.0;	    
	  }
	}
	else{
	  state = STATE_REST;
	  rest_timer = search_timer = track_timer = 0.0;	  
	}
	
	break;
      }
	
	
      // -----------------------------------------------------------------------------------------------------------
      default:
      {
	ROS_INFO ("Unknown state %d", state);
	state = STATE_REST;
	rest_timer = search_timer = track_timer = 0.0;	
	break;
      }
    }
    
    
    // Publish
    std_msgs::Bool msg;
    msg.data = (state == STATE_TRACK);
    pubTcpTracking.publish(msg);
    
    // Publish only when there is a new command!
    if (ardrone_commanded){
//       pubArdroneTwist.publish(commandTwist);
    }
    
//     ROS_INFO ("State %d", state);
//     ROS_INFO("predicted pose:  %f %f %f | %f", predictedArdronePose.x, predictedArdronePose.y, predictedArdronePose.z, predictedArdronePose.yaw);
    
    // Reset variables
    newDesiredPoseReceived = false;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // File 
  outFile.close();

  return 0;
}


Eigen::VectorXd computeDroneVelocityCommand(const double t_time_diff,
					    const tf::Transform t_TF_drone_tcp, const tf::Transform t_pastTF_drone_tcp, 
					    const tf::Transform t_desired_TF_drone_tcp, const tf::Transform t_pastDesired_TF_drone_tcp){
  
    // Current pose
    double x = t_TF_drone_tcp.getOrigin().x();
    double y = t_TF_drone_tcp.getOrigin().y();
    double z = t_TF_drone_tcp.getOrigin().z();
    double t = Zangle_uXDrone_uZTCP(t_TF_drone_tcp.getBasis().getColumn(2));
    // Past pose
    double x_1 = t_pastTF_drone_tcp.getOrigin().x();
    double y_1 = t_pastTF_drone_tcp.getOrigin().y();
    double z_1 = t_pastTF_drone_tcp.getOrigin().z();
    double t_1 = Zangle_uXDrone_uZTCP(t_pastTF_drone_tcp.getBasis().getColumn(2));
    
    // Current desired pose
    double rx = t_desired_TF_drone_tcp.getOrigin().x();
    double ry = t_desired_TF_drone_tcp.getOrigin().y();
    double rz = t_desired_TF_drone_tcp.getOrigin().z();
    double rt = Zangle_uXDrone_uZTCP(t_desired_TF_drone_tcp.getBasis().getColumn(2));
    // Past desired pose
    double rx_1 = t_pastDesired_TF_drone_tcp.getOrigin().x();
    double ry_1 = t_pastDesired_TF_drone_tcp.getOrigin().y();
    double rz_1 = t_pastDesired_TF_drone_tcp.getOrigin().z();
    double rt_1 = Zangle_uXDrone_uZTCP(t_pastDesired_TF_drone_tcp.getBasis().getColumn(2));
    
    rz = 1.5;
    rz_1 = 1.5;
    
    // Velocity
    double dx = (x - x_1)/t_time_diff;
    double dy = (y - y_1)/t_time_diff;
    double dz = (z - z_1)/t_time_diff;
    double dt = (t - t_1)/t_time_diff;
    // Desired velocity
    double drx = (rx - rx_1)/t_time_diff;
    double dry = (ry - ry_1)/t_time_diff;
    double drz = (rz - rz_1)/t_time_diff;
    double drt = (rt - rt_1)/t_time_diff;
  
    // Error
    double ex = rx - x;
    double ey = ry - y;
    double ez = rz - z;
    double et = rt - t;
    // Past error
    double ex_1 = rx_1 - x_1;
    double ey_1 = ry_1 - y_1;
    double ez_1 = rz_1 - z_1;
    double et_1 = rt_1 - t_1;
    
    // Error derivative
// 	  double dex = (ex - ex_1)/t_time_diff;
// 	  double dey = (ey - ey_1)/t_time_diff;
// 	  double dez = (ez - ez_1)/t_time_diff;
// 	  double det = (et - et_1)/t_time_diff;
    double dex = drx - dx;
    double dey = dry - dy;
    double dez = drz - dz;
    double det = drt - dt;
    
    // Input
    //  x
    double kpx = 0.3;
    double kdx = 0.05;
    double ux = -(kpx * ex + kdx * dex);
    //  y
    double kpy = 0.3;
    double kdy = 0.05;
    double uy = -(kpy * ey + kdy * dey);
    //  z
    double kpz = 2.0;
    double kdz = 1.0;
    double uz = -(kpz * ez + kdz * dez);
    //  t
    double kpt = 0.15;
    double kdt = 0.0;
    double ut = -(kpt * et + kdt * det);

    
    Eigen::VectorXd input_values(4);
    input_values << ux, uy, uz, ut;
    input_values = signalFilterOutput(input_values, 4);
    ux = input_values(0);
    uy = input_values(1);
    uz = input_values(2);
    ut = input_values(3);
    
    std::cout << "------------------------------------------------"<< std::endl;
    std::cout << "rot: " << std::endl;
    std::cout <<t_desired_TF_drone_tcp.getBasis().getRow(0).x()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(0).y()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(0).z()<<" " << std::endl;
    std::cout <<t_desired_TF_drone_tcp.getBasis().getRow(1).x()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(1).y()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(1).z()<<" " << std::endl;
    std::cout <<t_desired_TF_drone_tcp.getBasis().getRow(2).x()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(2).y()<<" " <<t_desired_TF_drone_tcp.getBasis().getRow(2).z()<<" " << std::endl;
    std::cout << "t/rt: " <<t<<" / " <<rt<< std::endl;
    std::cout << "dt/drt: " <<dt<<" / " <<drt<< std::endl;	  
    std::cout << "ut: " <<et<<" " <<det<< std::endl;	  

    return input_values;
}


double Zangle_uXDrone_uZTCP(const tf::Vector3 vector){
    
    tf::Vector3 vn = tf::Vector3(-vector.x(), -vector.y(), 0.0).normalized();
  
    return sgn(vn.y())*acos(vn.x());
}


tf::Matrix3x3 cross_product_matrix_S(const tf::Vector3 vector){
    return tf::Matrix3x3(0.0, vector.z(), -vector.y(), -vector.z(), 0.0, vector.x(), vector.y(), -vector.x(), 0.0);
}


Eigen::VectorXd signalFilterOutput(const Eigen::VectorXd src, const unsigned int dim){

    //      B   b_0 + b_1*z^{-1} + ... + b_{nz}*z^{-nz}  
    //  H = - = ---------------------------------------
    //      A   a_0 + a_1*z^{-1} + ... + a_{np}*z^{-np}
    // nz = F_ORDER_ZEROS
    // np = F_ORDER_POLES

    //   z ||  0 | -1 | ... |           -n  
    // ------------------------------------
    //  xv ||  0 |  1 | ... | F_ORDER_POLES+1 
    //  yv ||  0 |  1 | ... | F_ORDER_ZEROS+1
    
    n_samples++;
    
    if (n_samples == 1)		loadFilterValues(a, F_ORDER_ZEROS, b, F_ORDER_POLES);
        
    Eigen::VectorXd out(dim);
    for (unsigned int nj=0; nj<dim; nj++){
    
        // Update history
        if (n_samples == 1){	// Initial values set to zero
            for (unsigned int i=0; i<F_ORDER_POLES; i++)	xv[i][nj] = 0.0;
            for (unsigned int i=0; i<F_ORDER_ZEROS; i++)	yv[i][nj] = 0.0;
        }
        for (unsigned int i=F_ORDER_POLES; i>=1; i--)	xv[i][nj] = xv[i-1][nj];
        xv[0][nj] = src(nj);
        for (unsigned int i=F_ORDER_ZEROS; i>=1; i--)	yv[i][nj] = yv[i-1][nj];

        // Compute new output
        double y_0 = 0.0;	// Reset output
        for (unsigned it=0; it<=F_ORDER_POLES; it++)	y_0 += b[it]*xv[it][nj];    // inputs
        for (unsigned it=1; it<=F_ORDER_ZEROS; it++)	y_0 -= a[it]*yv[it][nj];    // outputs
        y_0 *= 1.0/a[0];
        yv[0][nj] = out(nj) = y_0;
    }
                                                                                                                                                                                                                                                                                                                                                
    return out;
}

void loadFilterValues(double a[], const unsigned int aDim, double b[], const unsigned int bDim){

    // Butterworth 1rst Order - Cut-off frecuency = 1 Hz
    a[0]=1.00000000000000000000;    a[1]=-0.99373647154161459660;    b[0]=0.00313176422919270170;    b[1]=0.00313176422919270170;
  
    // Butterworth 1rst Order - Cut-off frecuency = 2 Hz
//    a[0]=1.00000000000000000000;    a[1]=-0.98751192990731428978; b[0]=0.00624403504634285511;    b[1]=0.00624403504634285511;  
  
    // Butterworth 1rst Order - Cut-off frecuency = 15 Hz
//    a[0]=1.00000000000000000000;    a[1]=-0.90992998817773751430; b[0]=0.04503500591113124285;    b[1]=0.04503500591113124285;
 
    // Butterworth 1rst Order - Cut-off frecuency = 30 Hz
//    a[0]=1.00000000000000000000;      a[1]=-0.82727194597247555308; b[0]=0.08636402701376222346;    b[1]=0.08636402701376222346;

    // Butterworth 1rst Order - Cut-off frecuency = 50 Hz
//    a[0]=1.00000000000000000000;    a[1]=-0.72654252800536101020;   b[0]=0.13672873599731949490;    b[1]=0.13672873599731949490;     
    
    // Butterworth 1rst Order - Cut-off frecuency = 100 Hz MASSA RÀPID
//    a[0]=1.00000000000000000000;    a[1]=-0.50952544949442879485;   b[0]=0.24523727525278560258;    b[1]=0.24523727525278560258;        
    
    return;
}