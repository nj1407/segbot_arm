#include <ros/ros.h>
#include <signal.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <jaco_msgs/JointAngles.h>
#include <sensor_msgs/JointState.h>

bool g_caught_sigint = false;

geometry_msgs::PoseStamped pose_current;
sensor_msgs::JointState js_cur;

std::vector<double> angles_current;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	pose_current = msg;
}
//Joint state cb
void joint_state_cb(const sensor_msgs::JointState &input){
	angles_current.clear();
	for(int i = 0; i < 6; i++){
		angles_current.push_back(input.position.at(i));
	}
	js_cur = input;
}

// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

/*
 * Prints the first 6 joints of a joint state message
 * No formatting, or rounding
 */
void printJointState(sensor_msgs::JointState js){
	ROS_INFO("Q1: %f, Q2: %f, Q3: %f, Q4: %f, Q5: %f, Q6: %f",
		js.position.at(0), js.position.at(1),js.position.at(2),js.position.at(3),js.position.at(4),js.position.at(5));
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "mico_kinematic_demo");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
	signal(SIGINT, sig_handler);

	// Start a service client
	ros::ServiceClient fkine_client = node_handle.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
	ros::ServiceClient ikine_client = node_handle.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
	ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 1);
	
	//publish FK pose
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("fkine/pose", 10);
	
	//make controller service
	//ros::ServiceClient client = node_handle.serviceClient<moveit_utils::MicoController>("mico_controller");

	ros::Subscriber sub_tool = node_handle.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	ros::Subscriber sub_angles = node_handle.subscribe ("/joint_states", 1, joint_state_cb);

	while(!fkine_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}

	/*
	 * Forward Kinematic service call
	 */

	moveit_msgs::GetPositionFK::Request fkine_request;
	moveit_msgs::GetPositionFK::Response fkine_response;


	ROS_INFO("Grabbing current joint state for comparison.");
	ros::spinOnce();
	sensor_msgs::JointState q_true = js_cur;

	//Load request with the desired link
	fkine_request.fk_link_names.push_back("mico_end_effector");

	//and the current frame
	fkine_request.header.frame_id = "mico_link_base";

	//finally we let moveit know what joint positions we want to compute
	//in this case, the current state
	fkine_request.robot_state.joint_state = q_true;

	ROS_INFO("Making FK call");
 	if(fkine_client.call(fkine_request, fkine_response)){
 		pose_pub.publish(fkine_response.pose_stamped.at(0));
 		ROS_INFO("Call successful. Pose published.");
 	} else {
 		ROS_INFO("Call failed. Terminating.");
 		ros::shutdown();
 		return 1;
 	}

 	ROS_INFO("Please move the arm to a different position to test inverse kinematics");
 	pressEnter();

 	/*
 	 * Inverse Kinematic call
 	 */

 	moveit_msgs::GetPositionIK::Request ikine_request;
	moveit_msgs::GetPositionIK::Response ikine_response;
	ikine_request.ik_request.group_name = "arm";
	ikine_request.ik_request.pose_stamped.header.frame_id = fkine_response.pose_stamped.at(0).header.frame_id;
	ikine_request.ik_request.pose_stamped.pose = fkine_response.pose_stamped.at(0).pose;

	/* Call the service */
	if(ikine_client.call(ikine_request, ikine_response)){
		ROS_INFO("IK service call success");
		ROS_INFO("Original q:");
		printJointState(q_true);
		ROS_INFO("Computed q from initial fKine: ");
		printJointState(ikine_response.solution.joint_state);
	} else {
		ROS_INFO("IK service call FAILED. Exiting");
	}
	//ROS_INFO_STREAM("Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ") << service_response.error_code.val);
	
	/* Filling in a seed state */
	/*robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("left_arm");
	*/
	/* Get the names of the joints in the right_arm*/
	//ikine_request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();
	
	/* Get the joint values and put them into the message, this is where you could put in your own set of values as well.*/
	//kinematic_state->setToRandomPositions(joint_model_group);
	//kinematic_state->copyJointGroupPositions(joint_model_group, service_request.ik_request.robot_state.joint_state.position);
	
	/* Call the service again*/
	//ikine_client.call(service_request, service_response);
	//ROS_INFO_STREAM("Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ") << service_response.error_code.val);
	
	/* Check for collisions*/
	//service_request.ik_request.avoid_collisions = true;
	
	/* Call the service again*/
	//ikine_client.call(service_request, service_response);
	//ROS_INFO_STREAM("Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ") << service_response.error_code.val);

	ros::shutdown();
	return 0;
}
