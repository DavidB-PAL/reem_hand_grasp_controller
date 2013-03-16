### ROS Node: reem_hand_grasp_controller
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>
<br>
A grasp controller and Action Server for the REEM robot hand.

This node provides an all-in-one posture controller for the REEM hand, and can be used directly as an Action Server for the hand, whilst also being compatible with the ROS Manipulation Pipeline.

Works with both simulated or real robot. In simulation, the controller moves all 3 joints of each finger with some delay, to mimic the behaviour of the underactuated joints on the real robot. 

*** This node is very verbose, and prints lots of debug information, to help with tweaking the parameters.
<br>

<br>
Creates a ROS Action Server at <br>
&nbsp;&nbsp;&nbsp;&nbsp;/Node_Namespace/grasp_posture_controller/ <br>
so it should be launched under the same namespace as the hand controllers, <br>
such that running 2 nodes gives e.g. <br>
&nbsp;&nbsp;&nbsp;&nbsp;/left_hand_controller/grasp_posture_controller/ <br>
&nbsp;&nbsp;&nbsp;&nbsp;/right_hand_controller/grasp_posture_controller/ <br>

It receives Action messages of type <br>
&nbsp;&nbsp;&nbsp;&nbsp;object_manipulation_msgs::GraspHandPostureExecutionAction <br>
containing one of three goals <br>
&nbsp;&nbsp;&nbsp;&nbsp;object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP <br>
&nbsp;&nbsp;&nbsp;&nbsp;object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP <br>
&nbsp;&nbsp;&nbsp;&nbsp;object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE <br>
along with  angles...

and sends commands to the JointTrajectoryAction controller in its own namespace, <br>
using a non-blocking Action call to <br>
&nbsp;&nbsp;&nbsp;&nbsp;.../follow_joint_trajectory/ <br>
and monitors the state of the controller using a callback on the topic <br>
&nbsp;&nbsp;&nbsp;&nbsp;.../state
<br>

<br>
The thumb is moved during the pre-grasp phase, using position control.

In the grasp phase, there are 3 modes of operation (configured by launch file): <br>
&nbsp;&nbsp;&nbsp;&nbsp;- position control <br>
&nbsp;&nbsp;&nbsp;&nbsp;- position stall checking <br>
&nbsp;&nbsp;&nbsp;&nbsp;- velocity stall checking (not finished) <br>
The simulated robot will close the hand until the average change of position of all 3 joints in each finger is below a threshold. <br>
The result is that each finger will stop independently if it can't move any further. <br>
The real robot works in the same way, but the controller only checks the position of the main encoder for each finger.

*** Note: Currently position control is enabled for the real robot. Position stall checking has stopped working due to a problem with reading the positions from the encoders. <br>
Velocity stall checking is designed to do the same thing by using the raw velocity values directly from the controller, but currently this information is not available.

In the release phase, the controller uses position control and opens the hand in 2 steps, to allow the 
object to fall away from the thumb before the thumb is retracted. 
<br>

<br>
ToDo: The velocity control mode needs to be re-tested on the real robot - the fingers stop just after they start moving, so the position data needs to be checked, and the position averaging method checked. <br>
Change the launch file so can easily use the same parameters for left & right controller.

This node mimics the interface of 'pr2_gripper_grasp_controller', which wraps 'pr2_gripper_action'.
<br>

<br>
**Required ROS packages:** <br>
reem_common     (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_simulation (DavidB-PAL fork, not yet merged with Master 15/3/13) 
<br>

<br>
**Usage (simulation):** <br>
$ roslaunch reem_gazebo reem_empty_world.launch <br>
Load the controller with velocity control of the 6 finger joints: <br>
$ roslaunch reem_hand_grasp_controller both_hands.launch sim:=true <br>

**Usage (real robot):** <br>
Connect to the robot. <br>
Load the controller with position control of the 2 finger motors: <br>
$ roslaunch reem_hand_grasp_controller both_hands.launch sim:=false 
<br>

<br>
**Testing:** <br>
You may want to move the arm away from the body. <br>
Watch the simulated or real robot, and execute the following test commands: <br>
$ rosrun reem_hand_grasp_controller grasp_left <br>
$ rosrun reem_hand_grasp_controller release_left <br>
or <br>
$ rosrun reem_hand_grasp_controller grasp_right <br>
$ rosrun reem_hand_grasp_controller release_right <br>

This grasp controller node prints lots of debugging output, <br>
or to get status from the JointTrajectoryAction controller: <br>
$ rostopic echo /left_hand_controller/state






