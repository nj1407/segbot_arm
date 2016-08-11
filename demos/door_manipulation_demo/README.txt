Read Me:

Running/testing:

To do the entire thing as a single file run

roslaunch door_manipulation_demo door_maniuplation_demo.launch

It will launch the action

For the action on its own

For the action server
roslaunch door_manipulation_demo door_open_as

For the action client
roslaunch door_manipulation_demo door_open_client

For the visualization of the point cloud 
roslaunch door_manipulation_demo door_handle_detection

For the nona action version 
roslaunch door_manipulation_demo door_open 

instead of the action sever and client 

Description:
door_handle_detection.cpp  - Service that takes the cloud and does filtering euclidian clustering after planar segmentation
and to detect the door handle taking the largest cluster and generates a centroid of the centroid and publishes it

door_open.cpp - Suscribes to the published point of door_handle centroid and generates an array of points from it 
to arrpoach the door and then go forwards afterwards (a total of two poses one to approach and one to actually push)
and does inverse kinematics to check if it possible to do so and if so actually makes the movements with using the planar
and publising linear velocities as a fail safe. Then it does another service call of door_handle_detection.cpp and compares 
the planar coefficents to see if the door has actually moved.

door_open_as.cpp and door_open_client.cpp - Are action versions of the code above

Using the actually code:
The code works best when run closest to the door;however, it is possible to call one go_to_door nodes that faces the door
in the code base and it will work if the approach point for the door is close enough for the door. It is advised to only
do this code with doors you know that are not spring locked/heavy since the arm doesn't have the force to be able to push
these doors. The code only works if the door is slightly open and will fail if it is closed always and if it is known
the door is CLOSED it is HIGHLY SUGGESTED NOT TO RUN THE CODE. This has been tested only a few times to confirm failures
and if done autonomously without supervision could lead to complications if the move it planar takes large/longer paths.
This code works best when the arm donesn't have to make large angular movements/ if the door handle is closer to the arm
with a very high success rate if the door is open but still works if this is not true with a reduced success rate.

TODO:
It currently isn't consitant if the robot can navigate through the door after this code has been run once. Need to add
a check if it can navigate through the door and if not add an approach node similar to the one used to approach a table
and then rerun the code if the planar coefficents have changed
