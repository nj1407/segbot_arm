#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>    
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/impl/transforms.hpp>

//including package services 
#include "elevator_press_button/color_perception.h"

// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

//tf 
#include <tf/transform_listener.h>
#include <tf/tf.h>
#define PI 3.14159265

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_centroid (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;
geometry_msgs::Quaternion plane_coeff;

sensor_msgs::PointCloud2 cloud_ros;

// used to see if door has moved
sensor_msgs::JointState current_state;
Eigen::Vector4f plane_coefficients_compare;
bool firstLook = true;

ros::Publisher goal_pub;
ros::Publisher cloud_pub;
ros::Publisher elevator_cloud_pub;
ros::Publisher debug_pub;
ros::Publisher plane_coeff_pub;
ros::Publisher goal_pub_lower;

//edit
ros::Publisher filtered_img1;
ros::Publisher filtered_img2;
ros::Publisher filtered_img3;
ros::Publisher filtered_img4;
ros::Publisher filtered_img5;
ros::Publisher filtered_img6;
//end edit

PointCloudT::Ptr cloud_costmap (new PointCloudT);


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

double plane_distance_tolerance = 0.08;
double plane_max_distance_tolerance = 0.03;
double cluster_extraction_tolerance = 0.02;

bool collecting_cloud = false;

// Check if a file exist or not
bool file_exist(std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

/* what happens when ctr-c is pressed */
void sig_handler(int sig)
{
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

// used to get current state to see if door has moved
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
    //NUM JOINTS IS 8
    if (input->position.size() == 8){
        current_state = *input;
    }
}

void computeClusters(PointCloudT::Ptr in, double tolerance){
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance); // 5cm
    ROS_INFO("tolerance %f points.",tolerance);
    //meant to include everything
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (in);
    ec.extract (cluster_indices);
    clusters.clear();
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (in->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
        cloud_mutex.lock ();
        //convert to PCL format
        pcl::fromROSMsg (*input, *cloud);
        //state that a new cloud is available
        new_cloud_available_flag = true;
        cloud_mutex.unlock ();
}

bool filter(PointCloudT::Ptr blob, Eigen::Vector4f plane_coefficients, double tolerance_min, double tolerance_max){
    double min_distance = 1000.0;
    double max_distance = -1000.0;
    //first, we find the point in the blob closest to the plane
    for (unsigned int i = 0; i < blob->points.size(); i++){
        pcl::PointXYZ p_i;
        p_i.x=blob->points.at(i).x;
        p_i.y=blob->points.at(i).y;
        p_i.z=blob->points.at(i).z;
        double distance = pcl::pointToPlaneDistance(p_i, plane_coefficients);
        if (distance < min_distance){
            min_distance = distance;
        }
        if (distance > max_distance){
            max_distance = distance;
        }
    }
    if (min_distance > tolerance_min)
        return false;
    else if (max_distance < 0.8*tolerance_max)
        return false;   
    ROS_INFO("\nMin Distance to plane for cluster with %i points: %f",(int)blob->points.size(),min_distance);
    ROS_INFO("Max Distance to plane for cluster with %i points: %f",(int)blob->points.size(),max_distance);
    return true;
}

void waitForCloud(){
    ros::Rate r(30);
    collecting_cloud = true;
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
        if (new_cloud_available_flag){
            new_cloud_available_flag = false;
            break;
        }
    }
    collecting_cloud = false;
}

/* collects a cloud by aggregating k successive frames */
void waitForCloudK(int k){
    ros::Rate r(30);
    cloud_aggregated->clear();
    int counter = 0;
    collecting_cloud = true;
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
        if (new_cloud_available_flag){
            *cloud_aggregated+=*cloud;
            new_cloud_available_flag = false;
            counter ++;
            if (counter >= k){
                cloud_aggregated->header = cloud->header;
                break;
            }
        }
    }
    collecting_cloud = false;
}

 PointCloudT::Ptr color_filter(int rMax, int rMin, int gMax, int gMin, int bMax, int bMin) {
	 
	ros::NodeHandle n; 
	 
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin))); 
    
	// build the filter 
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond); 
    condrem.setInputCloud (cloud); 
    condrem.setKeepOrganized(true); 
    PointCloudT::Ptr cloud_filtered;
    // apply filter 
    condrem.filter(*cloud_filtered); 
    ROS_INFO("Found %i points.",(int)cloud_filtered->points.size ());
	return cloud_filtered;
 }




bool seg_cb(elevator_press_button::color_perception::Request &req, elevator_press_button::color_perception::Response &res)
{    
    ROS_INFO("entered seg_cb");
    //get the point cloud by aggregating k successive input clouds
    waitForCloudK(15);
    cloud = cloud_aggregated;
    ROS_INFO("got cloud");

	PointCloudT filter1 = *color_filter(236, 217, 217, 203, 129, 53);
	PointCloudT filter2 = *color_filter(213, 176, 40, 23, 40, 21);
	PointCloudT filter3 = *color_filter(235, 212, 230, 198, 226, 195);
	PointCloudT filter4 = *color_filter(206, 114, 224, 191, 241, 216);
	PointCloudT filter5 = *color_filter(41, 20, 174, 87, 149, 67);
	PointCloudT filter6 = *color_filter(254, 140, 224, 97, 230, 90);
	elevator_cloud_pub.publish(filter1);
	
	

}   
    
int main (int argc, char** argv){
    //tf mico_link_base
    // Initialize ROS
    ros::init (argc, argv, "segbot_arm_button_handle_detector");
    ros::NodeHandle n;
    
    // Create a ROS subscriber for the input point cloud
    std::string param_topic = "/xtion_camera/depth_registered/points";
    ros::Subscriber sub = n.subscribe (param_topic, 1, cloud_cb);
    
    //create subscriber to joint angles
    ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
    
    ros::ServiceClient client = n.serviceClient<elevator_press_button::color_perception>("/pcl_button_filter/color_perception");
    
    //debugging publisher
    debug_pub = n.advertise<sensor_msgs::PointCloud2>("elevator_detector/debug2", 1);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("elevator_detector/debug", 1);
    elevator_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("elevator_detector/plane_cloud", 1);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal_to_go", 1);
    goal_pub_lower = n.advertise<geometry_msgs::PoseStamped>("goal_to_go_two", 1);
    filtered_img1 = n.advertise<sensor_msgs::PointCloud2>("yellow_filter/colorfil", 1);
    //service
    ros::ServiceServer service = n.advertiseService("pcl_button_filter/color_perception", seg_cb);
    //refresh rate
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);
    
    // Main loop:
    while (!g_caught_sigint && ros::ok()){  
        //collect messages
        ros::spinOnce();
        r.sleep();
    }

};
