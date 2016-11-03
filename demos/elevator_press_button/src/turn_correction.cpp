#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


//tf stuff
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

// PCL specific includes
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "elevator_press_button/color_perception.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <geometry_msgs/TwistStamped.h>

//the action definition
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/Odometry.h>

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

//for playing sounds when backing up
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//any table further than this away from the sensor will not be seen (in m)
#define FILTER_Z_VALUE 1.5
  
ros::NodeHandle nh_;
// NodeHandle instance must be created before this line. Otherwise strange error may occur.
sensor_msgs::JointState current_state;
nav_msgs::Odometry current_odom;
bool heard_odom;
bool new_cloud_available_flag = false;
ros::Publisher pub_base_velocity;
ros::Publisher pose_pub;
ros::Publisher sound_pub;

//used to compute transforms
tf::TransformListener tf_listener;

ros::Subscriber sub_odom_;
boost::mutex cloud_mutex;
//holds set of predefined positions
ArmPositionDB *posDB;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;
PointCloudT::Ptr cloud (new PointCloudT);

//odom state cb
void odom_cb(const nav_msgs::OdometryConstPtr& input){
    current_odom = *input;
    heard_odom = true;
}

void spinSleep(double duration){
    int rateHertz = 40; 
    ros::Rate r(rateHertz);
    for(int i = 0; i < (int)duration * rateHertz; i++) {
        ros::spinOnce();
        r.sleep();
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
    
double getYaw(geometry_msgs::Pose pose){
    tf::Quaternion q(pose.orientation.x, 
                            pose.orientation.y, 
                            pose.orientation.z, 
                            pose.orientation.w);
    tf::Matrix3x3 m(q);
    
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
    //NUM JOINTS IS 8
    if (input->position.size() == 8){
        current_state = *input;
    }
}

bool executeCB(elevator_press_button::color_perception::Request &req, elevator_press_button::color_perception::Response &res){
        
        Eigen::Vector4f plane_coef_vector;
        for (int i = 0; i < 4; i ++)
            plane_coef_vector(i)=res.cloud_plane_coef[i];

        //next, make arm safe to move again
        bool safe = segbot_arm_manipulation::makeSafeForTravel(nh_);
        if (!safe) {
            ROS_ERROR("[segbot_table_approach_as.cpp] Cannot make arm safe for travel! Aborting!");
            return false;
        }
        
        //transform clound into base_link frame of reference
        sensor_msgs::PointCloud cloud_pc1;
        sensor_msgs::convertPointCloud2ToPointCloud(res.cloud_plane,cloud_pc1);
        tf_listener.waitForTransform(res.cloud_plane.header.frame_id, "/base_footprint", ros::Time(0.0), ros::Duration(3.0)); 
        
        sensor_msgs::PointCloud transformed_cloud;
        tf_listener.transformPointCloud("/base_footprint",cloud_pc1,transformed_cloud); 
                
        //convert back to sensor_msgs::PointCloud2 and then to pcl format
        sensor_msgs::PointCloud2 plane_cloud_pc2;
        sensor_msgs::convertPointCloudToPointCloud2(transformed_cloud,plane_cloud_pc2);
        
        PointCloudT::Ptr cloud_plane (new PointCloudT);
        pcl::fromROSMsg(plane_cloud_pc2, *cloud_plane);
        
        //find the point on the table closest to the robot's 0,0
        int closest_point_index = -1;
        double closest_point_distance = 0.0;
        for (int i = 0; i < (int)cloud_plane->points.size(); i++){
            double d_i = sqrt(   pow(cloud_plane->points.at(i).x,2) +   pow(cloud_plane->points.at(i).y,2));
            if (closest_point_index == -1 || d_i < closest_point_distance){
                closest_point_index = i;
                closest_point_distance = d_i;
            }
        }
        
        geometry_msgs::PoseStamped pose_debug;
        pose_debug.header.frame_id = "/base_footprint";
        pose_debug.header.seq = 1;
        pose_debug.pose.position.x = cloud_plane->points.at(closest_point_index).x;
        pose_debug.pose.position.y = cloud_plane->points.at(closest_point_index).y;
        pose_debug.pose.position.z = cloud_plane->points.at(closest_point_index).z;
        pose_debug.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        pose_pub.publish(pose_debug);
        
        //calculate turn angle
        double target_turn_angle = atan2( cloud_plane->points.at(closest_point_index).y, cloud_plane->points.at(closest_point_index).x);
        
        
        double duration = 2.0; //we want to take this many seconds to get there
        double pub_rate = 30;
        
        double turn_velocity = 0.5*target_turn_angle/duration;
        
        ros::Rate r(pub_rate);
        
        //first, wait for odometry
        heard_odom = false;
        while (!heard_odom){
            r.sleep();
            ros::spinOnce();
        }
        
        double initial_yaw = getYaw(current_odom.pose.pose);
        double target_yaw = initial_yaw + target_turn_angle;
        
        ROS_INFO("Turn angle = %f",target_turn_angle);
        
        
        geometry_msgs::Twist v_i;
        v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
        v_i.angular.x = 0; v_i.angular.y = 0;
        
        //for (int i = 0; i < (int)duration*pub_rate;i++){
        while (ros::ok()){
            v_i.angular.z = turn_velocity;
            //ROS_INFO_STREAM(v_i);
            
            pub_base_velocity.publish(v_i);
            
            ros::spinOnce();
            
            r.sleep();
            
            double current_yaw = getYaw(current_odom.pose.pose);
            
            if (fabs(current_yaw - target_yaw) < 0.05)
                break;
        }
         
        v_i.angular.z = 0;
        pub_base_velocity.publish(v_i);
        
        heard_odom = false;
        while (!heard_odom){
            r.sleep();
            ros::spinOnce();
        }
        double final_yaw = getYaw(current_odom.pose.pose);
        
       return true;

}
int main (int argc, char** argv){
    //tf mico_link_base
    // Initialize ROS
    ros::init (argc, argv, "turn_correction");
    ros::NodeHandle n;;

    // Create a ROS subscriber for the input point cloud
    std::string param_topic = "/xtion_camera/depth_registered/points";
    ros::Subscriber sub = n.subscribe (param_topic, 1, cloud_cb);
    
    //create subscriber to joint angles
    ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

    //publisher for debugging purposes
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/segbot_table_approach_as/approach_table_target_pose", 1);
    
    //used to publish sound requests
    sound_pub = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);
    
    //velocity publisher
    pub_base_velocity = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    //load database of joint- and tool-space positions
    std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
    std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
    
    posDB = new ArmPositionDB(j_pos_filename, c_pos_filename);
    //service
    ros::ServiceServer service = n.advertiseService("pcl_button_filter/color_perception", executeCB);
    
    //refresh rate
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);
    
   //step 1: query table_object_detection_node to segment the blobs on the table
    posDB->print();
    //first, we need to move the arm out of view so the camera can see the table
    if (posDB->hasCarteseanPosition("side_view")){
        geometry_msgs::PoseStamped out_of_view_pose = posDB->getToolPositionStamped("side_view","/mico_link_base");
        //now go to the pose
        segbot_arm_manipulation::moveToPoseMoveIt(nh_,out_of_view_pose);
    }
    ros::ServiceClient srv = n.serviceClient<elevator_press_button::color_perception>("color_perception", executeCB);
    ros::ServiceClient client_panel = n.serviceClient<elevator_press_button::color_perception>("/pcl_button_filter/color_perception");
    elevator_press_button::color_perception panel_srv;  
        if (client_panel.call(panel_srv))
        {
            ROS_INFO("Received Response");
        }
        else
        {
            ROS_ERROR("Failed to call perception service");
            return false;
        }
        
        Eigen::Vector4f plane_coef_vector;
        for (int i = 0; i < 4; i ++)
            plane_coef_vector(i)=panel_srv.response.cloud_plane_coef[i];

        
        //next, make arm safe to move again
        bool safe = segbot_arm_manipulation::makeSafeForTravel(nh_);
        if (!safe) {
            ROS_ERROR("[segbot_table_approach_as.cpp] Cannot make arm safe for travel! Aborting!");
            return false;
        }
        
        //transform clound into base_link frame of reference
        sensor_msgs::PointCloud cloud_pc1;
        sensor_msgs::convertPointCloud2ToPointCloud(panel_srv.response.cloud_plane,cloud_pc1);
        tf_listener.waitForTransform(panel_srv.response.cloud_plane.header.frame_id, "/base_footprint", ros::Time(0.0), ros::Duration(3.0)); 
        
        sensor_msgs::PointCloud transformed_cloud;
        tf_listener.transformPointCloud("/base_footprint",cloud_pc1,transformed_cloud); 
                
        //convert back to sensor_msgs::PointCloud2 and then to pcl format
        sensor_msgs::PointCloud2 plane_cloud_pc2;
        sensor_msgs::convertPointCloudToPointCloud2(transformed_cloud,plane_cloud_pc2);
        
        PointCloudT::Ptr cloud_plane (new PointCloudT);
        pcl::fromROSMsg(plane_cloud_pc2, *cloud_plane);
        
        //find the point on the table closest to the robot's 0,0
        int closest_point_index = -1;
        double closest_point_distance = 0.0;
        for (int i = 0; i < (int)cloud_plane->points.size(); i++){
            double d_i = sqrt(   pow(cloud_plane->points.at(i).x,2) +   pow(cloud_plane->points.at(i).y,2));
            if (closest_point_index == -1 || d_i < closest_point_distance){
                closest_point_index = i;
                closest_point_distance = d_i;
            }
        }
        
        geometry_msgs::PoseStamped pose_debug;
        pose_debug.header.frame_id = "/base_footprint";
        pose_debug.header.seq = 1;
        pose_debug.pose.position.x = cloud_plane->points.at(closest_point_index).x;
        pose_debug.pose.position.y = cloud_plane->points.at(closest_point_index).y;
        pose_debug.pose.position.z = cloud_plane->points.at(closest_point_index).z;
        pose_debug.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        pose_pub.publish(pose_debug);
        
        //calculate turn angle
        double target_turn_angle = atan2( cloud_plane->points.at(closest_point_index).y, cloud_plane->points.at(closest_point_index).x);
        
        
        double duration = 2.0; //we want to take this many seconds to get there
        double pub_rate = 30;
        
        double turn_velocity = 0.5*target_turn_angle/duration;
        
        //first, wait for odometry
        heard_odom = false;
        while (!heard_odom){
            r.sleep();
            ros::spinOnce();
        }
        
        double initial_yaw = getYaw(current_odom.pose.pose);
        double target_yaw = initial_yaw + target_turn_angle;
        
        ROS_INFO("Turn angle = %f",target_turn_angle);
        
        
        geometry_msgs::Twist v_i;
        v_i.linear.x = 0; v_i.linear.y = 0; v_i.linear.z = 0;
        v_i.angular.x = 0; v_i.angular.y = 0;
        
        //for (int i = 0; i < (int)duration*pub_rate;i++){
        while (ros::ok()){
            v_i.angular.z = turn_velocity;
            //ROS_INFO_STREAM(v_i);
            
            pub_base_velocity.publish(v_i);
            
            ros::spinOnce();
            
            r.sleep();
            
            double current_yaw = getYaw(current_odom.pose.pose);
            
            if (fabs(current_yaw - target_yaw) < 0.05)
                break;
        }
         
        v_i.angular.z = 0;
        pub_base_velocity.publish(v_i);
        
        heard_odom = false;
        while (!heard_odom){
            r.sleep();
            ros::spinOnce();
        }
        double final_yaw = getYaw(current_odom.pose.pose);
            

};
