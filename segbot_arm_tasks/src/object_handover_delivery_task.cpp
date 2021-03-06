#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#include <segbot_arm_perception/segbot_arm_perception.h>


#include "bwi_kr_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>


#include <moveit_utils/MicoNavSafety.h>

#define NUM_JOINTS 8 //6+2 for the arm


//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJoinstState && heardPose)
			return;
		
		r.sleep();
	}
}


// Blocking call for user input
void pressEnter(std::string message){
	std::cout << message;
	while (true){
		char c = std::cin.get();
		if (c == '\n')
			break;
		else if (c == 'q'){
			ros::shutdown();
			exit(1);
		}
		else {
			std::cout <<  message;
		}
	}
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "object_handover_delivery_task");
	
	ros::NodeHandle n;
	
	//retrieve params
	ros::NodeHandle privateNode("~");
	std::string delivery_door;
	privateNode.param<std::string>("door",delivery_door,"d4_414a");
  
	ROS_INFO("Target location: %s",delivery_door.c_str());

	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
	
	//load database of joint- and tool-space positions
	std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB positionDB(j_pos_filename, c_pos_filename);
	positionDB.print();

	//store out of table view joint position -- this is the position in which the arm is not occluding objects on the table
	listenForArmData();
	
	//Open hand and move to home position
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::openHand();
	
	/*if (positionDB.hasJointPosition("handover_front")){
		std::vector<float> handover_position = positionDB.getJointPosition("handover_front");
		sensor_msgs::JointState handover_js = segbot_arm_manipulation::valuesToJointState(handover_position);
		segbot_arm_manipulation::moveToJointState(n,handover_js);
		pressEnter("Press [Enter]");
	}
	else {
		ROS_ERROR("handover_front position does not exist! Aborting...");
		return 1;
	}*/
	
	if (positionDB.hasCarteseanPosition("handover_front")){
		/*std::vector<float> handover_position = positionDB.getJointPosition("handover_front");
		sensor_msgs::JointState handover_js = segbot_arm_manipulation::valuesToJointState(handover_position);
		segbot_arm_manipulation::moveToJointState(n,handover_js);*/
		
		geometry_msgs::PoseStamped handover_pose = positionDB.getToolPositionStamped("handover_front","mico_link_base");
		
		segbot_arm_manipulation::moveToPoseMoveIt(n,handover_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(n,handover_pose);

		
		pressEnter("Press [Enter]");
	}
	else {
		ROS_ERROR("handover_front position does not exist! Aborting...");
		return 1;
	}
	

	//now receive object
	segbot_arm_manipulation::TabletopGraspGoal receive_goal;
	receive_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER_FROM_HUMAN;
	receive_goal.timeout_seconds = -1.0;
	
	ac_grasp.sendGoal(receive_goal);
	ac_grasp.waitForResult();
	
	//now make safe
	segbot_arm_manipulation::homeArm(n);
	bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
	pressEnter("Press [Enter] to proceed with navigation");
	//now travel: to do
	
	bwi_kr_execution::ExecutePlanGoal goal_asp;
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(delivery_door);
	
	actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
	client_asp.waitForServer();

	rule.body.push_back(fluent);
    goal_asp.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client_asp.sendGoalAndWait(goal_asp);
	
	
	
	
	//no home and let go
	segbot_arm_manipulation::homeArm(n);
	
	if (positionDB.hasJointPosition("handover_front")){
		std::vector<float> handover_position = positionDB.getJointPosition("handover_front");
		sensor_msgs::JointState handover_js = segbot_arm_manipulation::valuesToJointState(handover_position);
		segbot_arm_manipulation::moveToJointState(n,handover_js);
	}
	else {
		ROS_ERROR("handover_front position does not exist! Aborting...");
		return 1;
	}
	
	segbot_arm_manipulation::TabletopGraspGoal handover_goal;
	handover_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER;
	handover_goal.timeout_seconds = -1.0;
	
	ac_grasp.sendGoal(handover_goal);
	ac_grasp.waitForResult();

	
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::closeHand();
	segbot_arm_manipulation::makeSafeForTravel(n);
	
}
