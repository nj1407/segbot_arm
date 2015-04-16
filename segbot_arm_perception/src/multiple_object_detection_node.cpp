#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include "segbot_arm_perception/TableDetectionObjectExtraction.h"

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Select mode
const bool save_pl_mode = false;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);


sensor_msgs::PointCloud2 cloud_ros;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

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
}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	cloud_mutex.lock();

	//convert to PCL format
	pcl::fromROSMsg(*input, *cloud);

	//state that a new cloud is available
    new_cloud_available_flag = true;

	cloud_mutex.unlock();
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);

    tf::TransformListener listener;

	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);

	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("multiple_object_detection/cloud", 10);

    // Setup service client for table detection
    ros::ServiceClient table_srv_client = nh.serviceClient<segbot_arm_perception::TableDetectionObjectExtraction>("/segbot_arm_perception/table_detection_object_extraction_server");


	// Main loop:
	while (!g_caught_sigint && ros::ok()) {
		//collect messages
		ros::spinOnce();
		r.sleep();

        if (new_cloud_available_flag) {
            new_cloud_available_flag = false;
            toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.frame_id = cloud->header.frame_id;
            // ROS_INFO("Publishing full cloud...");
            // cloud_pub.publish(cloud_ros);
            ros::Duration(2).sleep();

            // Prepare service call message
            segbot_arm_perception::TableDetectionObjectExtraction table_srv;
            // Pack service request
            toROSMsg(*cloud, table_srv.request.cloud);
            ROS_INFO("Calling Service...");
            if (table_srv_client.call(table_srv) && table_srv.response.is_plane_found) {
                PointCloudT::Ptr cloud_plane(new PointCloudT);
                std::vector<PointCloudT::Ptr > clusters_on_plane;
                float cloud_plane_coef[4];
                ROS_INFO("Table found");
                // Retrieve values
                for (int i = 0; i < table_srv.response.cloud_plane_coef.size(); i++) {
                    cloud_plane_coef[i] = table_srv.response.cloud_plane_coef[i];
                }
                fromROSMsg(table_srv.response.cloud_plane, *cloud_plane);
                for (int i = 0; i < table_srv.response.cloud_clusters.size(); i++) {
                    PointCloudT::Ptr temp_ptr(new PointCloudT);
                    fromROSMsg(table_srv.response.cloud_clusters[i], *temp_ptr);
                    clusters_on_plane.push_back(temp_ptr);
                }
                // Do work
                // Create the normal estimation class, and pass the input dataset to it
                pcl::NormalEstimation<PointT, pcl::Normal> ne;
                ne.setInputCloud (cloud_plane);
                ROS_INFO("Hi");
                // Create an empty kdtree representation, and pass it to the normal estimation object.
                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
                ne.setSearchMethod (tree);

                // Output datasets
                pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
                ROS_INFO("Hi");
                // Use all neighbors in a sphere of radius 3cm
                ne.setRadiusSearch (0.03);

                // Compute the features
                ne.compute (*cloud_normals);
                ROS_INFO("Hi");
                // Check for undefined values
                for (int i = 0; i < cloud_normals->points.size(); i++)
                {
                    if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i]))
                    {
                        PCL_WARN("normals[%d] is not finite\n", i);
                    }
                }
                // Visualize
                // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                // viewer->addPointCloud<pcl::Normal> (cloud_normals, "normal cloud");
                ROS_INFO("Hi");
                // Create the PFH estimation class, and pass the input dataset+normals to it
                pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
                pfh.setInputCloud (cloud_plane);
                pfh.setInputNormals (cloud_normals);
                // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

                // Create an empty kdtree representation, and pass it to the PFH estimation object.
                // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
                pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
                //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
                pfh.setSearchMethod (tree2);
                ROS_INFO("Hi2");
                // Output datasets
                pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

                // Use all neighbors in a sphere of radius 5cm
                // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
                pfh.setRadiusSearch (0.05);
                ROS_INFO("Hi");
                // Compute the features
                pfh.compute (*pfhs);
                ROS_INFO("Hi");
                ROS_INFO("Publishing cloud clusters...");
                toROSMsg(*cloud_plane, cloud_ros);
                // toROSMsg(*cloud_normals, cloud_ros);
                cloud_pub.publish(cloud_ros);
            }
        }
	}
}
