#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <math.h>
#include <signal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// For traversing the filesystem
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp> 

#include "segbot_arm_perception/FeatureExtraction.h"

using namespace std;
using namespace boost::filesystem;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// General point cloud to store the whole image
PointCloudT::Ptr image_cloud (new PointCloudT);

// Fill in the cloud data
pcl::PCDReader reader;

// Total number of objects
int total_objects = 32;
    
// The general file path
std::string generalFilePath = "/home/users/pkhante/look_behaviour/";

// Filepath of the csv file
std::string data_file_path = "/home/users/pkhante/look_behaviour/extracted_feature_data.csv";

// Write the feature vector to disk in libSVM format.
// It also writes the object id corresponding to the feature to another file
bool write_feature_to_disk_csv(std::vector<double>& feature_vector,
                               std::string feature_object_index_and_trial_number) {
    // Set filename for each run of the program
    /*while (data_file_name.empty()) {
        ROS_INFO("Output file name: ");
        std::getline (std::cin, data_file_name);
    }*/

	std::ofstream write_feature_file;
    write_feature_file.open(data_file_path.c_str(), std::ios::out | std::ios::app);
    if (write_feature_file.is_open()) {
		write_feature_file << feature_object_index_and_trial_number << ",";
        for (int i = 0; i < feature_vector.size(); i++) {
            write_feature_file << feature_vector[i];
            if (i + 1 < feature_vector.size()) {
                write_feature_file << ",";
            }
        }
        write_feature_file << "\n";
        write_feature_file.close();
        //ROS_INFO("Finished writing to the file");
    } else {
        ROS_ERROR("Cannot open file");
    }

    return true;
}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "extract_object_features");
	ros::NodeHandle nh;
    
    // Setup service client for extracting the features
	ros::ServiceClient feature_srv_client = nh.serviceClient<segbot_arm_perception::FeatureExtraction>("/segbot_arm_perception/feature_extraction_server");
	int counter = 0;
	
    //File traversal so that all .pcd files in the above path are accessed consecutively
	for(int object_num = 1; object_num <= total_objects; object_num++){
		ROS_INFO("Inside for");
		std::stringstream convert1;
		convert1 << object_num;
		string folderPath = generalFilePath + "obj_" + convert1.str();
		ROS_INFO("FolderPath : %s", folderPath.c_str());
		path dir_path(folderPath);
		if(!exists(dir_path)){
			ROS_INFO("Exiting as folder does not exist");
			return 1;
		}
		    
		directory_iterator itr(dir_path), eod; 		
		BOOST_FOREACH(path const &p, std::make_pair(itr, eod)){ 
			if(is_regular_file(p)){
				image_cloud->clear();
				string filePath = itr->path().string();
				ROS_INFO("Filepath: %s", filePath.c_str());
 				reader.read(filePath, *image_cloud);
 					
				std::size_t found = p.filename().string().find("_");
 				string obj_and_trial_num = p.filename().string().substr(0, found+7);
 				ROS_INFO("Object and trial number: %s", obj_and_trial_num.c_str());
 					
 				segbot_arm_perception::FeatureExtraction feature_srv;
				toROSMsg(*image_cloud, feature_srv.request.cloud);
				ROS_INFO("Calling feature extraction service...");
				// Find features of the object 
				if (feature_srv_client.call(feature_srv)) {
					ROS_INFO("Feature vector received");
					std::vector<double> feature_vector = feature_srv.response.feature_vector;
					// Write feature to disk
					write_feature_to_disk_csv(feature_vector, obj_and_trial_num);
				} else {
					ROS_WARN("Error: No clusters found or failed to retrieve feature vector");
				}
 			}
 		}
	}
}