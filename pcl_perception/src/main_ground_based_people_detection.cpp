/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */
 
#include <signal.h> 
#include <vector>
#include <string.h>
   
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
#include "utils/file_io.h"
#include "utils/viz_utils.h" 
#include "utils/Grouper.h"
#include "utils/pcl_utils.h"
 
//the srv
#include "pcl_perception/PeopleDetectionSrv.h"
 
enum node_states {IDLE, DETECTING};
int node_state = IDLE; 
const int num_frames_for_detection = 15;
int current_frame_counter = 0;
 
//some constants
bool visualize = false;
bool calibrate_plane = false;

const std::string data_topic = "nav_kinect/depth_registered/points"; 
const std::string classifier_location = "/home/bwi/catkin_ws/src/bwi_experimental/pcl_perception/data/classifier.yaml";
const std::string plane_coefs_location = "/home/bwi/catkin_ws/src/bwi_experimental/pcl_perception/data/ground_plane_avg.txt";
const std::string node_name = "segbot_people_detector";

float min_confidence = -1.6;//-1.8;//-1.9
float min_height = 1.3;
float max_height = 2.3;
float voxel_size = 0.06;
Eigen::Matrix3f rgb_intrinsics_matrix;

//some parameters for filtering out false positives
float max_distance_to_ground = 1.5; 


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//store the plane coefficients for the ground in kinnect frame of reference
Eigen::VectorXf ground_coeffs;


// Mutex: //
boost::mutex cloud_mutex;


bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);

enum { COLS = 640, ROWS = 480 };

//some publishers
ros::Publisher marker_pub;

// viewer
pcl::visualization::PCLVisualizer *viewer_display;          

//data structures for detecting people
pcl::people::PersonClassifier<pcl::RGB> person_classifier;
pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object

//struct for a measurement
struct PersonDetectionMeasurement {
	Eigen::Vector3f centroid;
	int frame_index;
};



void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//ROS_INFO("Heard cloud: %s",input->header.frame_id.c_str());
	
	cloud_mutex.lock (); 
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;
	
	cloud_mutex.unlock ();
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void calibrate_floor()
{
	ROS_INFO("[main_ground_based_people_detection.cpp] Waiting for next cloud...");
	while(!new_cloud_available_flag) {
		//collect messages
		ros::spinOnce();
	}

	ROS_INFO("[main_ground_based_people_detection.cpp] Heard first cloud, calibrating ground plane...");
	  
	new_cloud_available_flag = false;

	cloud_mutex.lock ();    // for not overwriting the point cloud

	// Display pointcloud:
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
		
	// PCL viewer //
	pcl::visualization::PCLVisualizer viewer_calibrate("PCL Viewer");
	viewer_calibrate.addPointCloud<PointT> (cloud, rgb, "input_cloud");
	viewer_calibrate.setCameraPosition(0,0,-2,0,-1,0,0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d (new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_calibrate);
	viewer_calibrate.registerPointPickingCallback (pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	// Spin until 'Q' is pressed:
	viewer_calibrate.spin();
	std::cout << "done." << std::endl;
	  
	cloud_mutex.unlock ();    

	// Ground plane estimation:
	std::vector<int> clicked_points_indices;
	for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
		clicked_points_indices.push_back(i);
	pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
	model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
	
	//save to file
	write_vector_to_file("ground_plane.txt",ground_coeffs,4);
	
}

bool service_callback(pcl_perception::PeopleDetectionSrv::Request  &req,
         pcl_perception::PeopleDetectionSrv::Response &res)
{
	ROS_INFO("Service requested with command: %s",req.command.c_str());
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
		
	viewer_display->removeAllShapes();
	
	//the node cannot be asked to detect people while someone has asked it to detect people
	if (node_state != IDLE){
		ROS_WARN("[segbot_pcl_person_detector] Person detection service may not be called until previous call is processed");
		return true;
	}
	
	//command for detecting people standing up
	if (req.command == "detect_people"){

		current_frame_counter = 0;
		bool detecting = true;
		node_state = DETECTING;
		
		//where to store all measurements while detecting
		std::vector<PersonDetectionMeasurement> measurements;
		
		while (detecting){
			
			if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
			{
				ROS_INFO("Detecting on frame %i",current_frame_counter);
				
				new_cloud_available_flag = false;

				// Perform people detection on the new cloud:
				std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
				std::vector<pcl::people::PersonCluster<PointT> > clusters_filtered;
				people_detector.setInputCloud(cloud);
				people_detector.setGround(ground_coeffs);                    // set floor coefficients
				people_detector.compute(clusters);                           // perform people detection

				ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

				// Draw cloud and people bounding boxes in the viewer:
				if (visualize){
					viewer_display->removeAllPointClouds();
					viewer_display->removeAllShapes();
				}
					
				
				if (visualize){
					pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
					viewer_display->addPointCloud<PointT> (cloud, rgb, "input_cloud");
				}
					
					
				//prepare vizualization message
				visualization_msgs::MarkerArray markers_msg;
					
				unsigned int k = 0;
				unsigned int c = 0;
				unsigned int draw_id = 0;
				for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
				{
					//ROS_INFO("Cluster %i: confidence: %f",k, it->getPersonConfidence());
					if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
					{
						Eigen::Vector3f centroid_k = it->getCenter();
						Eigen::Vector3f top_k = it->getTop();
						Eigen::Vector3f bottom_k = it->getBottom();	
						
						//calculate the distance from the centroid of the cloud to the plane
						pcl::PointXYZ p_k;
						
						p_k.x=bottom_k(0);p_k.y=bottom_k(1);p_k.z=bottom_k(2);
						double dist_to_ground_bottom = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
						p_k.x=top_k(0);p_k.y=top_k(1);p_k.z=top_k(2);
						double dist_to_ground_top = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
						p_k.x=centroid_k(0);p_k.y=centroid_k(1);p_k.z=centroid_k(2);
						double dist_to_ground = pcl::pointToPlaneDistance(p_k,ground_coeffs);
							
						ROS_INFO("Cluter centroid: %f, %f, %f",centroid_k(0),centroid_k(1),centroid_k(2));
						ROS_INFO("\tDistance to ground (top): %f",dist_to_ground_top);
						ROS_INFO("\tDistance to ground (centroid): %f",dist_to_ground);
						ROS_INFO("\tDistance to ground (bottom): %f",dist_to_ground_bottom);
						ROS_INFO("\tCluster height: %f",it->getHeight());
						ROS_INFO("\tCluster points: %i",it->getNumberPoints());
						ROS_INFO("\tDistance from sensor: %f",it->getDistance());	
						ROS_INFO("\tconfidence: %f",it->getPersonConfidence());	
						//	
							
							
						bool accept = true;
						
						if (it->getNumberPoints() < 250) //a person should have about 350 points +- 50 depending on distance from kinect
							accept = false;
						else if (it->getNumberPoints() > 600) //a person should have about 450 points +- 50 depending on distance from kinect
							accept = false;
						else if (it->getHeight() < 1.1) //nobody should be shorter than a meter and 10 cm
							accept = false;
						else if (it->getHeight() > 2.2) //or taller than 2.2 meters
							accept = false;
						if (dist_to_ground_bottom > 0.3) //or hovering more than 30 cm over the floor
							accept = false;
						
							
						if (accept){
							// draw theoretical person bounding box in the PCL viewer:
							if (visualize){
								it->drawTBoundingBox(*viewer_display, draw_id);
								draw_id ++;
								
							}
						
							ROS_INFO("Accepted!");
							
							//ROS_INFO("\tTTop and TBottom: %f, %f",
							
							visualization_msgs::Marker marker_k = create_next_person_marker(it,"/nav_kinect_rgb_optical_frame","segbot_pcl_person_detector",k);	
							markers_msg.markers.push_back(marker_k);

							k++;
							
							
							PersonDetectionMeasurement next_measurement;
							next_measurement.centroid = centroid_k;
							next_measurement.frame_index = current_frame_counter;
							measurements.push_back(next_measurement);
						}
						else {
							ROS_INFO("Rejected!");
							
							std::string false_positive_id = "sphere"+c;
							viewer_display->addSphere (p_k, 0.2, 1.0, 0, 0, false_positive_id);
							it->drawTBoundingBox(*viewer_display, draw_id);
							draw_id++;
						}
					}
						
					/*if (k > 0){
						 // Publish the markers
						marker_pub.publish(markers_msg);
					}*/
					
					c++;
				}

				std::cout << k << " people found" << std::endl;
					
				//increment counter and see if we need to stop detecting
				current_frame_counter++;
				if (current_frame_counter > num_frames_for_detection){
					detecting = false;
				}
			}
				
			if (visualize){
				viewer_display->spinOnce();
			}
		
			cloud_mutex.unlock ();
			
			//spin to collect next point cloud
			ros::spinOnce();
		}
		
		
		//group measurements into clusters
		pcl_perception::Grouper3F G;
		float dist_threshold = 0.3;
		int min_group_size = 3;
		for (unsigned int i = 0; i < measurements.size(); i ++){
			G.add_next(i,measurements.at(i).centroid,dist_threshold);
		}
		G.print();
		
		
		
		
		
		//trivial and bad solution -- just return the centroids of the group as poses
		for (unsigned int i = 0; i < G.groups.size(); i++){
			
			if ((int)G.groups.at(i).data.size() >= min_group_size){
				Eigen::Vector3f position_i = G.groups.at(i).mean;
				geometry_msgs::Pose pose_i;
				
				pose_i.position.x=position_i(0);
				pose_i.position.y=position_i(1);
				pose_i.position.z=position_i(2);
				pose_i.orientation = tf::createQuaternionMsgFromYaw(0);
				
				
				
				//test transforming pose from kinect to map frame of reference
				geometry_msgs::PoseStamped stampedPose;
				
				stampedPose.header.frame_id = "/nav_kinect_rgb_optical_frame";
		    	stampedPose.header.stamp = ros::Time(0);
		    	stampedPose.pose = pose_i;
		    	
		    	//transforms the frame
		    	geometry_msgs::PoseStamped stampOut;
		    	listener.waitForTransform("/nav_kinect_rgb_optical_frame", "/map", ros::Time(0), ros::Duration(3.0)); 
		        listener.transformPose("/map", stampedPose, stampOut);
		        
		        ROS_INFO("Transformed pose position: %f, %f, %f",stampOut.pose.position.x, stampOut.pose.position.y, stampOut.pose.position.z);
		    
				res.poses.push_back(stampOut);
		    
			}
		}
		
	}
		
	
	node_state = IDLE;
  
	/*res.sum = req.a + req.b;
	ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: [%ld]", (long int)res.sum);*/
	return true;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_pcl_person_detector");
	ros::NodeHandle nh;
	
	//retrieve any arguments from the launch file or parameter server
	nh.param<bool>("launch_pcl_viewer", visualize, false);
	nh.param<bool>("calibrate_ground_plane", calibrate_plane, false);
	 
	//initialize marker publisher
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("segbot_pcl_person_detector/marker", 1000);
	  
	  
	// Create a ROS subscriber for the input point cloud
	ROS_INFO("[main_ground_based_people_detection.cpp] Subscribing to point cloud topic...");
	ros::Subscriber sub = nh.subscribe (data_topic, 1, cloud_cb);
	
	//advertise service
	ros::ServiceServer detection_service = nh.advertiseService("segbot_pcl_person_detector/people_detection_service", service_callback);

	// set filename for classifier and intrinsics for camera
	std::string svm_filename = classifier_location;
	rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

	//register ctrl-c
	signal(SIGINT, sig_handler);


	
	//either calinrate floor or use existing calibration
	ground_coeffs.resize(4);
	if (true){
		calibrate_floor();
	}
	else {
		ground_coeffs = load_vector_from_file(plane_coefs_location.c_str(),4);
	}
	
	std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

	// Initialize new viewer:
	if (visualize){
		viewer_display = new pcl::visualization::PCLVisualizer("People Viewer"); 
		viewer_display->setCameraPosition(0,0,-2,0,-1,0,0);
	}

	// Create classifier for people detection: 
	ROS_INFO("[main_ground_based_people_detection.cpp] loading classifier..."); 
	person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

	// People detection app initialization:
	people_detector.setVoxelSize(voxel_size);                        // set the voxel size
	people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
	people_detector.setClassifier(person_classifier);                // set person classifier
	people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

	// For timing:
	static unsigned count = 0;
	static double last = pcl::getTime ();

	// Main loop:
	while (/*!viewer.wasStopped()*/ !g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		
		
	}

	return 0;
}

