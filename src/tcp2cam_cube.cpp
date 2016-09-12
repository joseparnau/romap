/*
  Software License Agreement (BSD License)

  Copyright (c) 2012, Scott Niekum
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

  author: Scott Niekum
  
    
  modified by:  Josep A. Claret
  date:         November 2015
*/


#include "ros/ros.h"
#include <std_msgs/Bool.h>

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>

using namespace alvar;
using namespace std;

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
tf::TransformBroadcaster *tf_broadcaster;
tf::Transform TCPToBundleTF, CamToBundleTF, TcpToCamTF;
MarkerDetector<MarkerData> marker_detector;
MultiMarkerBundle *multi_marker_bundle=NULL;
Pose CamToBundlePoses;
int master_id;
bool bundle_seen;
std::vector<int> bundle_indices; 	
bool init = true;  

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string WorldToCamTF_name;


void GetMultiMarkerPoses(IplImage *image);
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);


void GetMultiMarkerPoses(IplImage *image) {

  if (marker_detector.Detect(image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true))
  {
    multi_marker_bundle->Update(marker_detector.markers, cam, CamToBundlePoses);
    
    if(marker_detector.DetectAdditional(image, cam, false) > 0)
    {
      if ((multi_marker_bundle->SetTrackMarkers(marker_detector, cam, CamToBundlePoses, image) > 0))
	multi_marker_bundle->Update(marker_detector.markers, cam, CamToBundlePoses);
    }
  }
}



//Callback to handle getting video frames and processing them
void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_){
    try{
      
      // Scan image for tags and bundle from image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      IplImage ipl_image = cv_ptr_->image;
      GetMultiMarkerPoses(&ipl_image);
      
      // Check if a mark has been detected
      bundle_seen = false;
      for (size_t i=0; i<marker_detector.markers->size(); i++)
      {
	int id = (*(marker_detector.markers))[i].GetId();
	if(id >= 0)
	{
	  //Mark the bundle that marker belongs to as "seen"
	  for(int k=0; k<bundle_indices.size(); k++)
	  {
	    if(bundle_indices[k] == id)
	    {
	      bundle_seen = true;
	      break;
	    }
	  }
	}
      }
//       if (bundle_seen)	std::cout << "detect: YES" << std::endl;
//       else		std::cout << "detect: NO" << std::endl;
      
      
      if (bundle_seen)
      {
	// Get transformation from Camera to Bundle cube: T^CAM_BUNDLE
	double px,py,pz,qx,qy,qz,qw;
	px = CamToBundlePoses.translation[0]/100.0;
	py = CamToBundlePoses.translation[1]/100.0;
	pz = CamToBundlePoses.translation[2]/100.0;
	qx = CamToBundlePoses.quaternion[1];
	qy = CamToBundlePoses.quaternion[2];
	qz = CamToBundlePoses.quaternion[3];
	qw = CamToBundlePoses.quaternion[0];
	
// 	std::cout << "q: " <<qx<<", "<<qy<<", "<<qz<<", "<<qw<<"  = " << sqrt(qx*qx+ qy*qy + qz*qz + qw*qw) << std::endl;

	// Get the Bundle pose in the Camera frame
	tf::Quaternion CamToBundleTF_rotation (qx,qy,qz,qw);
	tf::Vector3 CamToBundleTF_origin (px,py,pz);
	tf::Transform CamToBundleTF (CamToBundleTF_rotation, CamToBundleTF_origin);
	
	// Compute transformation from Tcp to Camera: T^TCP_CAM = T^TCP_BUNDLE ( T^CAM_BUNDLE )^{-1}	
	TcpToCamTF = TCPToBundleTF * CamToBundleTF.inverse();
      }
      
//       tf::StampedTransform tcpToCamStTF (TcpToCamTF, image_msg->header.stamp, "/tcp", "/camera"); 
      tf::StampedTransform tcpToCamStTF (TcpToCamTF.inverse(), image_msg->header.stamp, "/camera", "/tcp");
      tf_broadcaster->sendTransform(tcpToCamStTF);     
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    }
  }
}



int main(int argc, char *argv[])
{
  ros::init (argc, argv, "tcp_to_cam_cube");
  ros::NodeHandle n;
  
  ros::Publisher detected_cube_pub = n.advertise<std_msgs::Bool>("tcp_detected", 10);
  
  ros::Rate loop_rate(50);  
  
  
  if(argc < 8)
  {
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./findMarkerBundles <marker size in cm> <max new marker error> <max track error> <cam image topic> <cam info topic> <output frame> <bundle XML file>" << endl;
    std::cout << std::endl;
    return 0;
  }

  // Get params from command line
  marker_size            = atof(argv[1]);
  max_new_marker_error   = atof(argv[2]);
  max_track_error        = atof(argv[3]);
  cam_image_topic        = argv[4];
  cam_info_topic         = argv[5];
  WorldToCamTF_name      = argv[6];
  int n_args_before_xml  = 7;
  marker_detector.SetMarkerSize(marker_size);
  
  // Initialize transformations
  
  //   Fixed: Bundle pose in the Tcp frame
  TCPToBundleTF.setRotation(tf::Quaternion(0,0,0,1));
  TCPToBundleTF.setOrigin(tf::Vector3(0,0,0.14)); 

  //   Fixed:Bundle pose in the Tcp frame
  CamToBundleTF.setRotation(tf::Quaternion(0,0,0,1));
  CamToBundleTF.setOrigin(tf::Vector3(0,0,0)); 
  
  TcpToCamTF = TCPToBundleTF * CamToBundleTF.inverse();

  
  // Load the marker bundle XML file
  CamToBundlePoses.Reset();
  MultiMarker loadHelper;
  if(loadHelper.Load(argv[n_args_before_xml], FILE_FORMAT_XML))
  {
    vector<int> id_vector = loadHelper.getIndices();
    multi_marker_bundle = new MultiMarkerBundle(id_vector);
    multi_marker_bundle->Load(argv[n_args_before_xml], FILE_FORMAT_XML);
    master_id = multi_marker_bundle->getMasterId();
    bundle_indices = multi_marker_bundle->getIndices();
  }
  else
  {
    cout<<"Cannot load file "<< argv[n_args_before_xml] << endl;
    return 0;
  }

  // Set up camera, listeners, and broadcasters
  cam = new Camera(n, cam_info_topic);
  tf_broadcaster = new tf::TransformBroadcaster();
  
//   tf::StampedTransform tcpToCamStTF (TcpToCamTF, ros::Time::now(), "/tcp", "/camera");   
//   tf_broadcaster->sendTransform(tcpToCamStTF);
  
//   tf_broadcaster->sendTransform(tf::StampedTransform(TcpToCamTF, ros::Time::now(), "/tcp", "/camera"));
  tf_broadcaster->sendTransform(tf::StampedTransform(TcpToCamTF.inverse(), ros::Time::now(), "/camera", "/tcp"));   
  
  //Give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();			
  
  //Subscribe to topics and set up callbacks
  ROS_INFO ("Subscribing to image topic");
  image_transport::ImageTransport it_(n);
  cam_sub_ = it_.subscribe (cam_image_topic, 1, &getCapCallback);

//   ros::spin();
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Bool cube_detected;
    cube_detected.data = bundle_seen;
    detected_cube_pub.publish(cube_detected);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
