
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

sensor_msgs::ImageConstPtr image; 
std::string path_to_image; 
SearchPlanner *planner;  // motion thread terminated when vision is done

/*---------------------------------------------------------------
        a           b           1, board near conference room
           m                    2, board near 400/500 doors
        c           d       
    M of coordinates(x,y) is inside the rectangle iff, 
    (0 < AM dot AB < AB dot AB) AND (0 < AM dot AC < AC dot AC) 
-----------------------------------------------------------------*/

bool ScavTaskWhiteBoard::inRectangle(my_pose* m, my_pose* a, my_pose* b, my_pose* c) 
{
    float am_ab = (m->x - a->x) * (b->x - a->x) + (m->y - a->y) * (b->y - a->y);
    float ab_ab = (b->x - a->x) * (b->x - a->x) + (b->y - a->y) * (b->y - a->y);
    float am_ac = (m->x - a->x) * (c->x - a->x) + (m->y - a->y) * (c->y - a->y);
    float ac_ac = (c->x - a->x) * (c->x - a->x) + (c->y - a->y) * (c->y - a->y);

    return (0 < am_ab and am_ab < ab_ab) and (0 < am_ac and am_ac < ac_ac); 
}

void callback_human_detected(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pose a1 = Pose(-30.88, 0.06);
    Pose b1 = Pose(-29.48, 0.06);
    Pose c1 = Pose(-30.89, -3.16);

    Pose a2 = Pose(-8.25, -5.99);
    Pose b2 = Pose(-6.83, -6.05);
    Pose c2 = Pose(-8.21, -11.26);

    Pose pose = Pose(msg->pose.position.x, msg->pose.position.y); 

    if (inRectangle(pose, a1, b1, c1) or inRectangle(pose, a2, b2, c2)) {

        ROS_INFO("People detected near a white board, picture saved.");

        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);

        boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time(); 
        path_to_image = directory + "white_board_" + boost::posix_time::to_simple_string(curr_time); 

        if (false == boost::filesystem::is_directory(directory)) {
            boost::filesystem::path tmp_path(directory);
            boost::filesystem::create_directory(tmp_path);
        } 
        cv::imwrite(file, cv_ptr->image);
        planner->setTargetDetection(true); // change status to terminate the motion thread
    }
}

void callback_image(const sensor_msgs::ImageConstPtr& msg) {
    image = msg;
}

void ScavTaskWhiteBoard::motionThread() {
    std::string path_to_yaml = ros::package::getPath("bwi_scavenger") + "/support/real.yaml";
    planner = new SearchPlanner(nh, path_to_yaml, 0.2);           

    int next_goal_index;                                                        
    while (ros::ok()) {
        planner->moveToNextScene( planner->selectNextScene(planner->belief, next_goal_index) );
        planner->analyzeScene(0.25*PI, PI/10.0);
        planner->updateBelief(next_goal_index);
    }
}

void ScavTaskWhiteBoard::visionThread() {

    ros::Subscriber sub_human = nh.subscribe("/segbot_pcl_person_detector/human_poses", 100, callback_human_detected); 
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/nav_kinect/rgb/image_color", 1, callback_image);

    ros::Rate r(10); 
    while (ros::ok() and r.sleep()) {
        ros::spinOnce(); 
    }
}

void ScavTaskWhiteBoard::executeTask(int timeout, TaskResult &result, std::string &record) {

    std::thread motion(this->motionThread);
    std::thread vision(this->visionThread); 

    motion.join();
    vision.join(); 
    record = path_to_image; 
    result = SUCCEDDED; 
}

