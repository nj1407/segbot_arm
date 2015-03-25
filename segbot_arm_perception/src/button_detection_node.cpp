
#include <signal.h> 
#include <vector>
#include <string>
#include <ros/ros.h>

#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>

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

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;
sensor_msgs::PointCloud2 cloud_ros;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
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
	
	
	cloud_mutex.lock (); 
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;
	
	cloud_mutex.unlock ();
}

double computeAvgRedValue(PointCloudT::Ptr in){
	double total_red = 0;
	
	for (unsigned int i = 0; i < in->points.size(); i++){
		total_red += in->points.at(i).r;
		
	}
	
	total_red /= (in->points.size());
	return total_red;
}

void computeClusters(PointCloudT::Ptr in, double tolerance){
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (in);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (tolerance); // 2cm
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

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_arm_button_detector");
	ros::NodeHandle nh;
	
	// Create a ROS subscriber for the input point cloud
	std::string param_topic = "/camera/depth_registered/points";
	ros::Subscriber sub = nh.subscribe (param_topic, 1, cloud_cb);
	
	//debugging publisher
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("button_detection_node/cloud", 10);
	  
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//refresh rate
	double ros_rate = 3.0;
	ros::Rate r(ros_rate);
	
	// Main loop:
	while (!g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
	
		r.sleep();
		
		if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
		{
			new_cloud_available_flag = false;
			
			//Step 1: z-filter
			
			// Create the filtering object
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (cloud);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0.0, 1.15);

			pass.filter (*cloud);
			
			
			
			//Step 2: plane fitting
			
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
			// Create the segmentation object
			pcl::SACSegmentation<PointT> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (1000);
			seg.setDistanceThreshold (0.01);
			
			// Create the filtering object
			pcl::ExtractIndices<PointT> extract;
			
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud);
			seg.segment (*inliers, *coefficients);
			
			// Extract the plane
			extract.setInputCloud (cloud);
			extract.setIndices (inliers);
			extract.setNegative (false);
			extract.filter (*cloud_plane);
			
			//extract everything else
			extract.setNegative (true);
			extract.filter (*cloud_blobs);
			
			//get the plane coefficients
			Eigen::Vector4f plane_coefficients;
			plane_coefficients(0)=coefficients->values[0];
			plane_coefficients(1)=coefficients->values[1];
			plane_coefficients(2)=coefficients->values[2];
			plane_coefficients(3)=coefficients->values[3];
			
			
			//Step 3: Eucledian Cluster Extraction
			computeClusters(cloud_blobs,0.04);
			
			clusters_on_plane.clear();
			
			for (unsigned int i = 0; i < clusters.size(); i++){
				Eigen::Vector4f centroid_i;
				pcl::compute3DCentroid(*clusters.at(i), centroid_i);
				pcl::PointXYZ center;
				center.x=centroid_i(0);center.y=centroid_i(1);center.z=centroid_i(2);
				
				double distance = pcl::pointToPlaneDistance(center, plane_coefficients);
				if (distance < 0.1 /*&& clusters.at(i).get()->points.size() >*/ ){
					clusters_on_plane.push_back(clusters.at(i));
					
				}
			}
			
			
			//Step 4: detect the button among the remaining clusters
			
			
			int max_index = -1;
			double max_red = 0.0;
			for (unsigned int i = 0; i < clusters_on_plane.size(); i ++){
				double red_i = computeAvgRedValue(clusters_on_plane.at(i));
				ROS_INFO("Cluster %i: %i points, red_value = %f",i,(int)clusters_on_plane.at(i)->points.size(),red_i);
				
				if (red_i > max_red){
					max_red = red_i;
					max_index = i;
				}
			}
			
			//publish  cloud
			
			pcl::toROSMsg(*clusters_on_plane.at(max_index),cloud_ros);
			cloud_ros.header.frame_id = cloud->header.frame_id;
			cloud_pub.publish(cloud_ros);	
			
			
			
			//unlock mutex
			cloud_mutex.unlock ();
		}
		
	}
};