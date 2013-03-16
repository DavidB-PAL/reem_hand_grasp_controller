/*
 *  Grasp controller and Action Server for the REEM robot hand
 *
 *  This node provides an all-in-one posture controller for the REEM hand,
 *  and can be used directly as an Action Server for the hand, 
 *  whilst also being compatible with the ROS Manipulation Pipeline.
 *  
 *  Works with both simulated or real robot. In simulation, the controller moves all
 *  3 joints of each finger with some delay, to mimic the behaviour of the underactuated
 *  joints on the real robot. 
 *
 *  *** This node is very verbose, and prints lots of debug information, to help with tweaking the parameters.
 *
 *
 *  Creates a ROS Action Server at
 *      /Node_Namespace/grasp_posture_controller/
 *  so it should be launched under the same namespace as
 *  the hand controllers, such that running 2 nodes gives e.g.
 *     /left_hand_controller/grasp_posture_controller/
 *     /right_hand_controller/grasp_posture_controller/
 *
 *  It receives Action messages of type
 *     object_manipulation_msgs::GraspHandPostureExecutionAction
 *  containing one of three goals
 *     object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP
 *     object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP
 *     object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE
 *  along with  angles...
 *
 *  and sends commands to the JointTrajectoryAction controller in its own
 *  namespace, using a non-blocking Action call to
 *                 .../follow_joint_trajectory/
 *  and monitors the state of the controller using a callback
 *  on the topic
 *                 .../state
 *
 *
 *  The thumb is moved during the pre-grasp phase, using position control.
 *  
 *  In the grasp phase, there are 3 modes of operation (configured by launch file):
 *    - position control
 *    - position stall checking
 *    - velocity stall checking (not finished)
 *  The simulated robot will close the hand until the average change of position of all 3 joints
 *  in each finger is below a threshold. The result is that each finger will stop independently if
 *  it can't move any further.
 *  The real robot works in the same way, but the controller only checks the position of the main
 *  encoder for each finger.
 *  
 *  ** Note: Currently position control is enabled for the real robot. Position stall checking has stopped
 *           working due to a problem with reading the positions from the encoders.
 *  
 *           Velocity stall checking is designed to do the same thing by using the raw velocity values
 *           directly from the controller, but currently this information is not available.
 *  
 *  In the release phase, the controller uses position control and opens the hand in 2 steps, to allow the 
 *  object to fall away from the thumb before the thumb is retracted. 
 *
 *
 *  Author: David Butterworth
 * 
 *  This node mimics the interface of 'pr2_gripper_grasp_controller', which wraps 'pr2_gripper_action'.
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// ToDo: maybe get the 3 joint index numbers during construction, by querying JointTrajAction

#include "ros/ros.h"

// Advanced Action Server, for incoming grasp commands
#include <actionlib/server/action_server.h> 
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>

// ROS Service for confirming grasp status back to pipeline, ideally would use a touch sensor
#include <object_manipulation_msgs/GraspStatus.h>

// Simple Action client, for sending commands to joint controller
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

// An extra Action server that was used for testing a touch sensor,
// when called it cancels the grasp action
#include "std_msgs/Bool.h"

class REEMHandGraspController
{
public:
    REEMHandGraspController();
    ~REEMHandGraspController();

private:
    // ROS Node Handle
    ros::NodeHandle root_nh_;
    ros::NodeHandle priv_nh_;

    // Action Server for incoming grasp commands
    typedef actionlib::ActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> GAS;
    typedef GAS::GoalHandle GoalHandle;
    GAS* action_server_;

    // Action Client for moving the gripper joints
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *gripper_traj_action_client_;

    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);
    void controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg);
    void watchdog(const ros::TimerEvent &e);

    // This Service is called by the Manipulation Pipeline to check if the object is really within the gripper.
    // On the PR2 this was a simple gripper position check, which makes some sense for an effort controller joint.
    // Currently it just returns true, because the joints could be in any position depending on the object size,
    // but you could use this to read a touch sensor value.
    bool serviceCallback(object_manipulation_msgs::GraspStatus::Request &request,
                         object_manipulation_msgs::GraspStatus::Response &response);

    void callbackTest(const std_msgs::Bool::ConstPtr &msg); // for cancel action signal test

    ros::Timer watchdog_timer_; // to prevent the action not returning

    // state of the JointTrajectoryAction controller
    ros::Subscriber sub_controller_state_;
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state_;

    // grasp query Service
    ros::ServiceServer query_srv_;

    bool has_active_goal_;
    GoalHandle active_goal_;
    ros::Time goal_received_;

    bool using_sim_;
    std::string left_or_right_;

    double position_threshold_pre_grasp_;
    double goal_time_pre_grasp_;

    bool use_position_threshold_grasp_; // check if GRASP is in position
    double position_threshold_grasp_;

    bool check_velocity_stalled_grasp_; // check if joints have stopped moving, during GRASP, can't use on real robot
    double stall_velocity_threshold_grasp_;
    double stall_timeout_grasp_;
    ros::Time last_movement_time_;
    double goal_time_grasp_;

    bool check_position_stalled_grasp_; // check if joint positions have stalled, during GRASP

    double position_stall_epsilon_grasp_; // simulated robot
    double position_stall_epsilon_index_grasp_;  // real robot
    double position_stall_epsilon_middle_grasp_; // real robot

    double position_stall_timeout_grasp_;
    double ps_prev_time_;
    double ps_prev_position_[7];
    bool index_finger_position_stalled_;
    bool middle_finger_position_stalled_;
    bool initialize_ps_prev_position_;
    bool estimate_actual_position_;
    double current_grasp_position_[7];

    double position_threshold_release_;
    double goal_time_release_;

    // For publishing joint positions, without using Action
    // currently used in the cancel goal callback, but can possibly remove it
    ros::Publisher pub_controller_command_;

    ros::Subscriber subscriberTest; // for sensor test

    // for calculating joint velocity, or actually average position change
    double index_finger_prev_positions[5];
    double middle_finger_prev_positions[5];

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reem_hand_grasp_controller");
    REEMHandGraspController node;
    ros::spin();

    return 0;
}

// Constructor
REEMHandGraspController::REEMHandGraspController() :
    root_nh_(""),
    priv_nh_("~"),
    has_active_goal_(false),
    initialize_ps_prev_position_(false)
{
    // Check for JointTrajectoryAction from hand controller
    std::string gripper_action_name = root_nh_.resolveName("gripper_action_name");
    gripper_traj_action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(gripper_action_name, true);
    while(!gripper_traj_action_client_->waitForServer(ros::Duration(2.0)) && root_nh_.ok())
    {
        ROS_INFO_STREAM("Waiting for Action Server on " << gripper_action_name);
    }
    if (!root_nh_.ok()) exit(0);
    ROS_INFO_STREAM("Connected to JointTrajectoryAction Server.");

    // Start grasp posture Action Server
    std::string posture_action_name = root_nh_.resolveName("posture_action_name");
    action_server_ = new actionlib::ActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction>(root_nh_, posture_action_name,
            boost::bind(&REEMHandGraspController::goalCB, this, _1),
            boost::bind(&REEMHandGraspController::cancelCB, this, _1),
            false ); // false prevents race condition
    action_server_->start();
    ROS_INFO_STREAM("REEM hand grasping posture Action Server started on " << posture_action_name);

    // Advertise grasp query Service
    std::string query_service_name = root_nh_.resolveName("grasp_query_name");
    query_srv_ = root_nh_.advertiseService(query_service_name, &REEMHandGraspController::serviceCallback, this);
    ROS_INFO_STREAM("REEM hand grasp query Service started on " << query_service_name);

    // Check if simulation or real robot
    priv_nh_.param<bool>("sim", using_sim_, false);
    std::string real_or_sim_string;
    if (using_sim_) real_or_sim_string = "(simulated robot, controlling 7 finger joints)";
    else            real_or_sim_string = "(real robot, controlling 3 under-actuated finger joints)";

    // Get position & velocity thresholds, these are dummy default values

    // Read parameters for PRE_GRASP action
    priv_nh_.param<double>("goal_time_pre_grasp", goal_time_pre_grasp_, 5.0);
    priv_nh_.param<double>("position_threshold_pre_grasp", position_threshold_pre_grasp_, 0.0);

    // Read parameters for GRASP action
    priv_nh_.param<double>("goal_time_grasp", goal_time_grasp_, 5.0);
    // (joint position checking)
    priv_nh_.param<bool>("use_position_threshold_grasp", use_position_threshold_grasp_, true); // enabled by default
    priv_nh_.param<double>("position_threshold_grasp", position_threshold_grasp_, 0.0);
    // (checking if joint positions don't change)
    priv_nh_.param<bool>("check_position_stalled_grasp", check_position_stalled_grasp_, false); // disabled by default
    priv_nh_.param<double>("position_stall_timeout_grasp", position_stall_timeout_grasp_, 0.0);

    priv_nh_.param<double>("position_stall_epsilon_grasp", position_stall_epsilon_grasp_, 0.0);

    priv_nh_.param<double>("position_stall_epsilon_index_grasp", position_stall_epsilon_index_grasp_, 0.0);
    priv_nh_.param<double>("position_stall_epsilon_middle_grasp", position_stall_epsilon_middle_grasp_, 0.0);

    priv_nh_.param<bool>("estimate_actual_position", estimate_actual_position_, false); // compensate for slow position feedback from real robot
    // (checking if stalled velocity)
    priv_nh_.param<bool>("check_velocity_stalled_grasp", check_velocity_stalled_grasp_, false); // disabled by default
    priv_nh_.param<double>("stall_velocity_threshold_grasp", stall_velocity_threshold_grasp_, 0.5);
    priv_nh_.param<double>("stall_timeout_grasp", stall_timeout_grasp_, 0.1); // this default is ok

    // Read parameters for RELEASE action
    priv_nh_.param<double>("goal_time_release", goal_time_release_, 5.0);
    priv_nh_.param<double>("position_threshold_release", position_threshold_release_, 0.0);

    // Get the hand name (left or right) from the Node's namespace
    // (the JointTrajectory msg we publish requires the finger joint names)
    std::string root_nh_namespace = ros::this_node::getNamespace();
    if ((root_nh_namespace.find("l_") != std::string::npos) || (root_nh_namespace.find("left_") != std::string::npos))
    {
        ROS_INFO("Starting REEM grasping controller for left hand %s", real_or_sim_string.c_str() );
        left_or_right_ = "left";
    }
    else if ((root_nh_namespace.find("r_") != std::string::npos) || (root_nh_namespace.find("right_") != std::string::npos))
    {
        ROS_INFO("Starting REEM grasping controller for right hand %s", real_or_sim_string.c_str() );
        left_or_right_ = "right";
    }
    else
    {
        ROS_ERROR("[reem_hand_grasp_controller]  You must launch this node under the same namespace as the hand controller");
        throw ros::Exception(""); // cancel loading node
    }

    // Can potentially remove this, and make the cancel callback call the goal cancel of the
    // joint trajectory controller
    pub_controller_command_ = root_nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    std::string command_topic_name = root_nh_.resolveName("grasp_query_name");
    ROS_INFO_STREAM("Subscribing to JointTrajectory controller on " <<command_topic_name);

    // Subscribe to state of JointTrajectoryAction controller
    sub_controller_state_ = root_nh_.subscribe("state", 1, &REEMHandGraspController::controllerStateCB, this);

    // for sensor test
    subscriberTest = root_nh_.subscribe("testsensor", 1, &REEMHandGraspController::callbackTest, this);
}

// Destructor
REEMHandGraspController::~REEMHandGraspController()
{
    // can potentially remove this
    pub_controller_command_.shutdown();

    // Delete subscriber, controller state
    sub_controller_state_.shutdown();
    // ?watchdog_timer_.stop();

    // Delete Action Client
    delete gripper_traj_action_client_;
}

// Watchdog, should cancel this Hand Action if it doesn't receive any
// callbacks from the JointTrajectoryAction server
//
// ToDo: maybe cancel if position not reached in x seconds, because if the grasp
//       hangs then it will block the pipeline.
void REEMHandGraspController::watchdog(const ros::TimerEvent &e)
{
    // Debug:
    printf("[reem_hand_grasp_controller]  called watchdog() \n");

    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
        bool should_abort = false;

        if (!last_controller_state_)
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we have never heard a controller state message.");

            // Debug:
            printf("[reem_hand_grasp_controller]  Aborting goal because we have never heard a controller state message. \n");
        }
        else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
        {
            should_abort = true;
            ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                     (now - last_controller_state_->header.stamp).toSec());

            // Debug:
            printf("[reem_hand_grasp_controller]  Aborting goal because we haven't heard from the controller in %.3lf seconds \n",
                   (now - last_controller_state_->header.stamp).toSec() );
        }

        if (should_abort)
        {
            // Marks the current goal as aborted.
            active_goal_.setAborted();
            has_active_goal_ = false;

            // Debug:
            printf("[reem_hand_grasp_controller]  watchdog marked call as aborted \n");

        }

    }
}

// callback for receiving a goal (PRE_GRASP, GRASP, or RELEASE)
void REEMHandGraspController::goalCB(GoalHandle gh)
{
    // Debug:
    //printf("[reem_hand_grasp_controller]  called goalCB()  \n");
    // printf("[reem_hand_grasp_controller]  called goalCB()   %d \n", action_server_->get_state()  );

    // Cancel the currently active goal
    if (has_active_goal_)
    {
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    goal_received_ = ros::Time::now();

    // Debug info:
    printf("[reem_hand_grasp_controller] called goalCB()  with Goal=%d  (1=PRE_GRASP, 2=GRASP, 3=RELEASE) \n", active_goal_.getGoal()->goal );

    // Prepare trajectory command for hand
    // don't use ros::Time::now() in header.stamp, this causes problems
    control_msgs::FollowJointTrajectoryGoal gripper_traj_command;
    gripper_traj_command.trajectory.points.resize(1); // 1 single waypoint
    if (!using_sim_)
    {
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
    }
    else
    {
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_2_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_3_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_2_joint");
        gripper_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_3_joint");
    }

    // Handle 3 gripper states, for goal types (1=PRE_GRASP, 2=GRASP, 3=RELEASE)
    switch (active_goal_.getGoal()->goal)
    {
    // GRASP Goal
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP:
        printf("   case GRASP \n");

        if (active_goal_.getGoal()->grasp.grasp_posture.position.empty() )
        {
            ROS_ERROR("[reem_hand_grasp_controller] position vector empty in requested grasp");
            active_goal_.setAborted();
            return;
        }
        if (active_goal_.getGoal()->grasp.grasp_posture.name.empty() )
        {
            ROS_ERROR("[reem_hand_grasp_controller] name vector empty in requested grasp");
            active_goal_.setAborted();
            return;
        }

        // Pass joint angles from posture msg into gripper trajectory
        // (the posture msg is defined with 7 joints in the hand YAML file)
        if (!using_sim_) // real robot
        {
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1] ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4] ); // middle1
        }
        else // simulated robot   // for index/middle, scale 4.5 radians max motion to 1.5
        {
            gripper_traj_command.trajectory.points.resize(3); // simulated grasp needs 3 waypoints, to mimic under-actuation

            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1]/3 ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.2*(active_goal_.getGoal()->grasp.grasp_posture.position[2])/3.0 );

            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0/3.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4]/3 ); // middle1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.2*(active_goal_.getGoal()->grasp.grasp_posture.position[5])/3.0 );

            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0/3.0 );

            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1]/3.0 ); // index1
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[2]/3.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[3]/2.0/3.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4]/3.0 ); // middle1
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[5]/3.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[6]/2.0/3.0 );

            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1]/3.0 ); // index1
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[2]/3.0 );
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[3]/3.0 );
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4]/3.0 ); // middle1
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[5]/3.0 );
            gripper_traj_command.trajectory.points[2].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[6]/3.0 );

            gripper_traj_command.trajectory.points[1].time_from_start = ros::Duration(1.5 * goal_time_grasp_); // velocity, 2nd joints in fingers
            gripper_traj_command.trajectory.points[2].time_from_start = ros::Duration(2.0 * goal_time_grasp_); // velocity, 3rd joints
        }
        // GRASP velocity (index_1_joint & middle_1_joint)
        gripper_traj_command.trajectory.points[0].time_from_start = ros::Duration(goal_time_grasp_); // goal_time_grasp_

        // Initialize variables for checking if joint positions are not changing
        ps_prev_time_ = (ros::Time::now()).toSec();
        index_finger_position_stalled_ = false;
        middle_finger_position_stalled_ = false;
        initialize_ps_prev_position_ = true;

        break;

    // PRE_GRASP Goal
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP:
        printf("   case PRE_GRASP \n");

        if (active_goal_.getGoal()->grasp.pre_grasp_posture.position.empty() )
        {
            ROS_ERROR("[reem_hand_grasp_controller] position vector empty in requested pre_grasp");
            active_goal_.setAborted();
            return;
        }
        if ( active_goal_.getGoal()->grasp.pre_grasp_posture.name.empty() )
        {
            ROS_ERROR("[reem_hand_grasp_controller] name vector empty in requested pre_grasp");
            active_goal_.setAborted();
            return;
        }

        // Pass joint angles from posture msg into gripper trajectory
        // (the posture msg is defined with 7 joints in the hand YAML file)
        if (!using_sim_) // real robot
        {
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.pre_grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // middle1
        }
        else  // simulated robot
        {
            gripper_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.pre_grasp_posture.position[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // middle1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
        }
        // PRE_GRASP velocity (thumb)
        gripper_traj_command.trajectory.points[0].time_from_start = ros::Duration(goal_time_pre_grasp_);

        break;

    // RELEASE Goal
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE:
        printf("   case RELEASE \n");

        if (!using_sim_) // real robot
        {
            gripper_traj_command.trajectory.points.resize(2); // add an extra point

            // Phase 1, dont move thumb (it might start open or closed), just open fingers first
            gripper_traj_command.trajectory.points[0].positions.push_back(  last_controller_state_->actual.positions[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // middle1

            // Phase 2, hand should be open
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // thumb
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // middle1
            gripper_traj_command.trajectory.points[1].time_from_start = ros::Duration(1.5+1.0); // velocity to open thumb
        }
        else // simulated robot
        {
            gripper_traj_command.trajectory.points.resize(2); // add an extra point

            // Phase 1, dont move thumb (it might start open or closed), just open fingers first
            gripper_traj_command.trajectory.points[0].positions.push_back( last_controller_state_->actual.positions[0] ); // thumb
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 ); // middle1
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[0].positions.push_back( 0.0 );

            // Phase 2, hand should be open
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // thumb
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // index1
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 ); // middle1
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[1].positions.push_back( 0.0 );
            gripper_traj_command.trajectory.points[1].time_from_start = ros::Duration(1.5+1.0); // velocity to open thumb
        }
        // RELEASE velocity (all joints)
        gripper_traj_command.trajectory.points[0].time_from_start = ros::Duration(1.5); // grasp_velocity_

        break;

    // unknown goal type, should never happen
    default:
        ROS_ERROR("[reem_hand_grasp_controller]  unknown goal code (%d) in goalCB() ", active_goal_.getGoal()->goal );
        active_goal_.setAborted();
        return;

    } // end switch

    // Send trajectory to hand controller
    gripper_traj_action_client_->sendGoal(gripper_traj_command);
    // non-blocking, don't wait for result
    // we check the status in the controllerStateCallback

    // Debug
    printf("[reem_hand_grasp_controller]  publishing trajectory command...  \n");

    last_movement_time_ = ros::Time::now();
}

// callback to cancel the Hand Action,
// publishes a new goal with current position
//
// ToDo: need to fix this so it can be used with simulated or real robot,
//       and maybe just use the cancel method, rather than publishing a new trajectory.
void REEMHandGraspController::cancelCB(GoalHandle gh)
{
    // Debug:
    printf("[reem_hand_grasp_controller]  called cancelCB() \n");

    if (active_goal_ == gh)
    {
        // Stops the controller.
        if (last_controller_state_)
        {
            // Get index number of a specific joint from the controller state message (which might have 2,3or7 joints)
            unsigned int joint_index = find( last_controller_state_->joint_names.begin(), last_controller_state_->joint_names.end(), "hand_"+left_or_right_+"_index_1_joint" )
                                       - last_controller_state_->joint_names.begin();

            // Stop the hand by sending a new traj with the current position
            // don't use ros::Time::now() in header.stamp, this causes problems
            trajectory_msgs::JointTrajectory hand_traj;
            hand_traj.points.resize(1);
            hand_traj.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
            hand_traj.points[0].positions.push_back( last_controller_state_->actual.positions[0] );
            hand_traj.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
            hand_traj.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
            for (unsigned int i=0; i<2; i++)
            {
                hand_traj.points[0].positions.push_back( last_controller_state_->actual.positions[joint_index] ); // goal is current position
            }

            // Set the velocity
            hand_traj.points[0].time_from_start = ros::Duration(0.0); // stop immediately

            pub_controller_command_.publish(hand_traj);

            //printf("[reem_hand_grasp_controller]  stopping at actual.position %f  \n", last_controller_state_->actual.positions[joint_index] );
            printf("[reem_hand_grasp_controller]  stopping...  \n");
            printf("   Position:  Index finger: %f  Middle finger: %f  \n", last_controller_state_->actual.positions[1], last_controller_state_->actual.positions[4] );
            printf("   Velocity:  Index finger: %f  Middle finger: %f  \n", last_controller_state_->actual.velocities[1], last_controller_state_->actual.velocities[4] );
        }

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }
}

// callback for state messages from the JointTrajectoryAction controller
void REEMHandGraspController::controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
{
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    // Callback is called a lot, so only continue if Action Goal is active
    if (!has_active_goal_)
        return;

    // Debug info:
    printf("[reem_hand_grasp_controller] called controllerStateCB()  with Goal=%d  (1=PRE_GRASP, 2=GRASP, 3=RELEASE) \n", active_goal_.getGoal()->goal );
    printf("    JointTrajectoryController state has %d joints: \n", msg->joint_names.size() );
    if (using_sim_) // Simulated robot, has position ratio x3
    {
        for (unsigned int i=0; i < msg->joint_names.size(); i++)
        {
            printf("    Joint %d (%s): desired.positions=%f   actual.positions=%f   actual.velocities=%f  \n", i, msg->joint_names[i].c_str(),
                   msg->desired.positions[i]*3.0, msg->actual.positions[i]*3.0, msg->actual.velocities[i] );
        }
    }
    else // Real robot
    {
        printf("    Joint %d (%s): desired.positions=%f   actual.positions=%f   actual.velocities=%f  \n", 1, msg->joint_names[1].c_str(),
               msg->desired.positions[1], msg->actual.positions[1], msg->actual.velocities[1] ); // index
        printf("    Joint %d (%s): desired.positions=%f   actual.positions=%f   actual.velocities=%f  \n", 2, msg->joint_names[2].c_str(),
               msg->desired.positions[2], msg->actual.positions[2], msg->actual.velocities[2] ); // middle
    }

    // temp vars
    double thumb_goal_angle;
    double index_goal_angle;
    double middle_goal_angle;
    unsigned int thumb_joint_index;
    unsigned int index_joint_index;
    unsigned int middle_joint_index;

    double index_joint_current_velocity; // should make these 2 an array
    double middle_joint_current_velocity;

    double current_position[7]; // check if goal position is reached
    double goal_position[7];

    // for checking if joint position is not changing
    bool update_index_finger_traj = false;
    bool update_middle_finger_traj = false;

    bool not_at_goal;
    bool a_joint_is_moving;
    bool cancel_goal;

    // Check if a goal state has been reached (1=PRE_GRASP, 2=GRASP, 3=RELEASE)
    switch (active_goal_.getGoal()->goal)
    {
    // GRASPING state
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP:


        // ToDo: swap all instances of *3 and /3 for simulated_joint_ratio variable.


        // this is first state calback for the GRASP action, so initialize array for checking if joint positions are not changing
        if (initialize_ps_prev_position_)
        {
            // initialize ps_prev_position_[] array with current positions of joints
            if (!using_sim_) // real robot
            {
                ps_prev_position_[0] = msg->actual.positions[0]; // thumb, not used
                ps_prev_position_[1] = msg->actual.positions[1]; // index
                ps_prev_position_[2] = msg->actual.positions[2]; // middle
            }
            else  // simulated robot
            {
                ps_prev_position_[0] = msg->actual.positions[0]*3.0; // thumb, not used
                ps_prev_position_[1] = msg->actual.positions[1]*3.0; // index
                ps_prev_position_[2] = msg->actual.positions[2]*3.0;
                ps_prev_position_[3] = msg->actual.positions[3]*3.0;
                ps_prev_position_[4] = msg->actual.positions[4]*3.0; // middle
                ps_prev_position_[5] = msg->actual.positions[5]*3.0;
                ps_prev_position_[6] = msg->actual.positions[6]*3.0;
            }
            initialize_ps_prev_position_ = false;
        }


        // GRASP Mode 1: Check if all joints are at goal positions
        if (use_position_threshold_grasp_)
        {
            // Debug info:
            printf("[reem_hand_grasp_controller]  Checking if joint positions within threshold for GRASP:   (position_threshold_grasp_=%f) \n", position_threshold_grasp_);
            if (using_sim_) // Simulated robot, has position ratio x3
            {
                for (unsigned int i=1; i<=6; i++)
                {
                    printf("Joint %s (%d)  position diff=%f  \n", msg->joint_names[i].c_str(), i,
                           fabs(msg->actual.positions[i]*3.0 - active_goal_.getGoal()->grasp.grasp_posture.position[i]) );
                }
            }
            else // Real robot
            {
                printf("Joint %s (%d)  pos diff=%f  \n", msg->joint_names[1].c_str(), 1,
                       fabs(msg->actual.positions[1] - active_goal_.getGoal()->grasp.grasp_posture.position[1]) ); // index
                printf("Joint %s (%d)  pos diff=%f  \n", msg->joint_names[2].c_str(), 2,
                       fabs(msg->actual.positions[2] - active_goal_.getGoal()->grasp.grasp_posture.position[4]) ); // middle
            }

            cancel_goal = false;
            if (using_sim_) // Simulated robot, 7 joints, with position ratio x3
            {
                // Check if all 6 finger joints are close to goal positions
                a_joint_is_moving = false;
                for (unsigned int i=1; i<=6; i++)
                {
                    if (fabs(msg->actual.positions[i]*3.0 - active_goal_.getGoal()->grasp.grasp_posture.position[i]) > position_threshold_grasp_)
                        a_joint_is_moving = true;
                }
                if (a_joint_is_moving == false)
                    cancel_goal = true;
            }
            else // Real robot, action.positions has 3 joints, but grasp_posture.positions (the goal) always has 7 joints
            {
                // check if index & middle finger are close to goal positions
                if ( (fabs(msg->actual.positions[1] - active_goal_.getGoal()->grasp.grasp_posture.position[1]) < position_threshold_grasp_) &&
                        (fabs(msg->actual.positions[2] - active_goal_.getGoal()->grasp.grasp_posture.position[4]) < position_threshold_grasp_) )
                {
                    cancel_goal = true;
                }
            }

            if (cancel_goal)
            {
                // Debug:
                printf("[reem_hand_grasp_controller] All joints are within position threshold (%f), the GRASP goal has been reached. \n", position_threshold_grasp_ );
                active_goal_.setSucceeded();
                has_active_goal_ = false;
            }

        } // endif use_position_threshold_grasp_


        // GRASP Mode 2: Check if joint positions have stalled
        else if (check_position_stalled_grasp_)
        {
            // Debug info:
            //printf("Debug check_position_stalled:   ps_prev_time=%f   now=%f \n", ps_prev_time_, (ros::Time::now()).toSec() );

            // if enough time has lapsed...
            if ( (fabs((ros::Time::now()).toSec() - ps_prev_time_)) > position_stall_timeout_grasp_)
            {
                printf("Checking for position stall...   time diff=%f  position_stall_timeout_grasp_=%f \n", ps_prev_time_-(ros::Time::now()).toSec(), position_stall_timeout_grasp_ );

                if (using_sim_) // Simulated robot, 7 joints, with position ratio x3
                {
                    // Debug info:
                    printf("                    position_stall_epsilon_grasp_ = %f \n", position_stall_epsilon_grasp_ );
                    printf("                    Joint 1 diff: %f   Joint 2 diff: %f   Joint 3 diff: %f \n", fabs(msg->actual.positions[1]*3.0 - ps_prev_position_[1]),
                           fabs(msg->actual.positions[2]*3.0 - ps_prev_position_[2]), fabs(msg->actual.positions[3]*3.0 - ps_prev_position_[3]));
                    printf("                    Joint 4 diff: %f   Joint 5 diff: %f   Joint 6 diff: %f \n", fabs(msg->actual.positions[4]*3.0 - ps_prev_position_[4]),
                           fabs(msg->actual.positions[5]*3.0 - ps_prev_position_[5]), fabs(msg->actual.positions[6]*3.0 - ps_prev_position_[6]));


                    printf("                    Index fingers average position diff: %f \n", ( (fabs(msg->actual.positions[1]*3.0 - ps_prev_position_[1])) +
                            (fabs(msg->actual.positions[2]*3.0 - ps_prev_position_[2])) + (fabs(msg->actual.positions[3]*3.0 - ps_prev_position_[3])) )/3.0  );
                    printf("                    Midle fingers average position diff: %f \n", ( (fabs(msg->actual.positions[4]*3.0 - ps_prev_position_[4])) +
                            (fabs(msg->actual.positions[5]*3.0 - ps_prev_position_[5])) + (fabs(msg->actual.positions[6]*3.0 - ps_prev_position_[6])) )/3.0  );

                    // Use average of the change of position of 3 joints in each finger, because the 1st joint moves a lot
                    // at the beginning, then the 2nd joint moves... etc.

                    // check if 3 index finger joints are in same position
                    if (!index_finger_position_stalled_ && ( ( ((fabs(msg->actual.positions[1]*3.0 - ps_prev_position_[1])) +
                            (fabs(msg->actual.positions[2]*3.0 - ps_prev_position_[2])) +
                            (fabs(msg->actual.positions[3]*3.0 - ps_prev_position_[3])) )/3.0) < position_stall_epsilon_grasp_
                                                           ))
                    {
                        // Debug:
                        printf("[reem_hand_grasp_controller]  By averaging, Index finger joints have all stopped moving. \n");

                        index_finger_position_stalled_ = true;
                        update_index_finger_traj = true;
                    }

                    // check if 3 middle finger joints are in same position
                    if (!middle_finger_position_stalled_ && ( (( (fabs(msg->actual.positions[4]*3.0 - ps_prev_position_[4])) +
                            (fabs(msg->actual.positions[5]*3.0 - ps_prev_position_[5])) +
                            (fabs(msg->actual.positions[6]*3.0 - ps_prev_position_[6])) )/3.0) < position_stall_epsilon_grasp_
                                                            ))
                    {
                        // Debug:
                        printf("[reem_hand_grasp_controller]  By averaging, Middle finger joints have all stopped moving. \n");

                        middle_finger_position_stalled_ = true;
                        update_middle_finger_traj = true;
                    }

                    // update previous time and joint positions
                    ps_prev_time_ = (ros::Time::now()).toSec();
                    for (unsigned int i=1; i<=6; i++) // 6 finger joints
                    {
                        ps_prev_position_[i] = msg->actual.positions[i]*3.0;
                    }

                }
                else // Real robot, independently check both finger encoders
                     // with different epsilon for each, because 1 finger is not moving not so good
                {
                    // Debug info:
                    //printf("                    position_stall_epsilon_grasp_ = %f \n", position_stall_epsilon_grasp_ );
                    printf("                    position_stall_epsilon_index_grasp_ = %f \n", position_stall_epsilon_index_grasp_ );
                    printf("                    Joint 1 diff: %f \n", fabs(msg->actual.positions[1] - ps_prev_position_[1]) );

                    printf("                    position_stall_epsilon_middle_grasp_ = %f \n", position_stall_epsilon_middle_grasp_ );
                    printf("                    Joint 2 diff: %f \n", fabs(msg->actual.positions[2] - ps_prev_position_[2]) );

                    // check if index finger joint is in same position
                    if (!index_finger_position_stalled_ && (fabs(msg->actual.positions[1] - ps_prev_position_[1]) < position_stall_epsilon_index_grasp_)) // own epsilon
                    {
                        // Debug:
                        printf("[reem_hand_grasp_controller]  Index finger motor has stopped moving. \n");

                        index_finger_position_stalled_ = true;
                        update_index_finger_traj = true;
                    }

                    // check if middle finger joint is in same position
                    if (!middle_finger_position_stalled_ && (fabs(msg->actual.positions[2] - ps_prev_position_[2]) < position_stall_epsilon_middle_grasp_)) // own epsilon
                    {
                        // Debug:
                        printf("[reem_hand_grasp_controller]  Middle finger motor has stopped moving. \n");

                        middle_finger_position_stalled_ = true;
                        update_middle_finger_traj = true;
                    }

                    // update previous time and joint positions
                    ps_prev_time_ = (ros::Time::now()).toSec();
                    ps_prev_position_[1] = msg->actual.positions[1]; // 2 finger joitns
                    ps_prev_position_[2] = msg->actual.positions[2];
                }

                // If either finger has stopped moving, send a modified trajectory for the whole hand
                if ((update_index_finger_traj) || (update_middle_finger_traj))
                {
                    // Firstly, create new trajectory msg with original GRASP goal positions
                    // don't use ros::Time::now() in header.stamp, this causes problems
                    control_msgs::FollowJointTrajectoryGoal updated_traj_command;
                    updated_traj_command.trajectory.points.resize(1); // 1 single waypoint
                    if (!using_sim_)
                    {
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1] ); // index1
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4] ); // middle1
                    }
                    else
                    {
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_2_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_3_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_2_joint");
                        updated_traj_command.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_3_joint");
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[0] ); // thumb
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[1]/3.0 ); // index1
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[2]/3.0 );
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[3]/3.0 );
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[4]/3.0 ); // middle1
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[5]/3.0 );
                        updated_traj_command.trajectory.points[0].positions.push_back( active_goal_.getGoal()->grasp.grasp_posture.position[6]/3.0 );
                    }
                    updated_traj_command.trajectory.points[0].time_from_start = ros::Duration(0.7); // velocity, magic number,  goal_time_grasp_

                    // Modify trajectory, stop the index finger
                    if (index_finger_position_stalled_)
                    {
                        // real robot, set index finger goal = current position
                        if (!using_sim_) 
                        {
                            // in ideal operation, can send the current position
                            if (!estimate_actual_position_)
                            {

                                updated_traj_command.trajectory.points[0].positions[1] = msg->actual.positions[1]; // index finger
                                // save position value
                                current_grasp_position_[1] = msg->actual.positions[1];

                                // Testing: read a different variable, doesnt seem to work any better on real robot
                                //updated_traj_command.trajectory.points[0].positions[1] = last_controller_state_->actual.positions[1]; 
                                //current_grasp_position_[1] = last_controller_state_->actual.positions[1];
                            }
                            // non-ideal, need to compensate for slow feedback, and guess the correct current position
                            else
                            {
                                // magic number, guess the actual position is 3.5 x the delayed position, the index finger is the worst
                                updated_traj_command.trajectory.points[0].positions[1] = (msg->actual.positions[1])*3.5; 
                                // save position value
                                current_grasp_position_[1] = (msg->actual.positions[1])*3.5; // same magic
                            }
                        }
                        // simulation, set index finger goal = current position
                        else
                        {
                            updated_traj_command.trajectory.points[0].positions[1] = msg->actual.positions[1]; // index finger 1
                            updated_traj_command.trajectory.points[0].positions[2] = msg->actual.positions[2]; // index finger 2
                            updated_traj_command.trajectory.points[0].positions[3] = msg->actual.positions[3]; // index finger 3
                            // save these position values
                            current_grasp_position_[1] = msg->actual.positions[1];
                            current_grasp_position_[2] = msg->actual.positions[2];
                            current_grasp_position_[3] = msg->actual.positions[3];
                        }
                    }
                    // Modify trajectory, stop the middle finger
                    if (middle_finger_position_stalled_)
                    {
                        // real robot, set  middle finger goal = current position
                        if (!using_sim_)
                        {
                            // in ideal operation, can send the current position
                            if (!estimate_actual_position_)
                            {
                                updated_traj_command.trajectory.points[0].positions[2] = msg->actual.positions[2]; // middle finger
                                // save position value
                                current_grasp_position_[2] = msg->actual.positions[2]; // same magic

                                // Testing: read a different variable, doesnt seem to work any better on real robot
                                //updated_traj_command.trajectory.points[0].positions[2] = last_controller_state_->actual.positions[2];
                                //current_grasp_position_[2] = last_controller_state_->actual.positions[2];
                            }
                            // non-ideal, need to compensate for slow feedback, and guess the correct current position
                            else
                            {
                                // magic number, guess the actual position is 2 x the delayed position, the middle finger is not so bad
                                updated_traj_command.trajectory.points[0].positions[2] = (msg->actual.positions[2])*2.0; // magic unicorn
                                // save position value
                                current_grasp_position_[2] = (msg->actual.positions[2])*2.0; // same magic
                            }
                        }
                        // simulation, set  middle finger goal = current position
                        else
                        {
                            updated_traj_command.trajectory.points[0].positions[4] = msg->actual.positions[4]; // middle finger 1
                            updated_traj_command.trajectory.points[0].positions[5] = msg->actual.positions[5]; // middle finger 2
                            updated_traj_command.trajectory.points[0].positions[6] = msg->actual.positions[6]; // middle finger 3
                            // save these position values
                            current_grasp_position_[4] = msg->actual.positions[4];
                            current_grasp_position_[5] = msg->actual.positions[5];
                            current_grasp_position_[6] = msg->actual.positions[6];
                        }
                    }

                    // Send trajectory to hand controller
                    gripper_traj_action_client_->sendGoal(updated_traj_command);

                    // Debug:
                    printf("debug:  sending updated trajectory... \n");

                    update_index_finger_traj = false;
                    update_middle_finger_traj = false;
                }

                // If both finger positions are stalled, the Action is complete
                if (index_finger_position_stalled_ && middle_finger_position_stalled_)
                {
                    // Debug:
                    printf("[reem_hand_grasp_controller]  Both fingers have stopped moving, the GRASP Action is complete. \n" );

                    active_goal_.setSucceeded();
                    has_active_goal_ = false;
                }
            }
        } // endif (check_position_stalled_grasp_)


        // GRASP Mode 3:  if velocity is below threshold, we must be grasping
        // 
        // For the simulation, we must check if all finger joints have stopped moving, because the 1st joints stop
        // before the 3rd,
        // but for real robot, the encoder is attached to the motor, so we can check when the motor has stalled.
        // 
        // *** Unfinished, this was originally tested and working on the simulated robot, but because velocity feedback
        // is not available from the real robot, both are now using a position stall check.
        // The code for the real robot was modified to attempt to derive velocity, but the position feedback is too slow for this.
        else if (check_velocity_stalled_grasp_)
        {
            if (using_sim_) // simulated robot
            {
                index_joint_current_velocity = fabs(msg->actual.velocities[index_joint_index]);
                middle_joint_current_velocity = fabs(msg->actual.velocities[middle_joint_index]);

                // Debug:
                printf("Joint %s (%d)  current velocity=%f  (stall velocity=%f)  \n", msg->joint_names[index_joint_index].c_str(), index_joint_index,
                       index_joint_current_velocity, stall_velocity_threshold_grasp_ );
                printf("Joint %s (%d)  current velocity=%f  (stall velocity=%f)  \n", msg->joint_names[middle_joint_index].c_str(), middle_joint_index,
                       middle_joint_current_velocity, stall_velocity_threshold_grasp_ );
            }

            else // Testing: real robot, average change of position, get velocity by deriving position
            {
                index_finger_prev_positions[0] = index_finger_prev_positions[1];
                index_finger_prev_positions[1] = index_finger_prev_positions[2];
                index_finger_prev_positions[2] = index_finger_prev_positions[3];
                index_finger_prev_positions[3] = index_finger_prev_positions[4];
                index_finger_prev_positions[4] = msg->actual.positions[index_joint_index];

                middle_finger_prev_positions[0] = middle_finger_prev_positions[1];
                middle_finger_prev_positions[1] = middle_finger_prev_positions[2];
                middle_finger_prev_positions[2] = middle_finger_prev_positions[3];
                middle_finger_prev_positions[3] = middle_finger_prev_positions[4];
                middle_finger_prev_positions[4] = msg->actual.positions[middle_joint_index];

                // sometimes get neg value
                index_joint_current_velocity = fabs( (index_finger_prev_positions[0] + index_finger_prev_positions[1] + index_finger_prev_positions[2]
                                                      + index_finger_prev_positions[3] + index_finger_prev_positions[4]) / 5.0 );
                middle_joint_current_velocity = fabs( (middle_finger_prev_positions[0] + middle_finger_prev_positions[1] + middle_finger_prev_positions[2]
                                                       + middle_finger_prev_positions[3] + middle_finger_prev_positions[4]) / 5.0 );

                // Debug:
                float t = (ros::Time::now() - last_movement_time_).toSec();
                printf("Joint %s (%d)  current avg velocity=%f  for t=%f  (stall velocity=%f)  \n", msg->joint_names[index_joint_index].c_str(), index_joint_index, t,
                       index_joint_current_velocity, stall_velocity_threshold_grasp_ );
                printf("Joint %s (%d)  current avg velocity=%f  for t=%f  (stall velocity=%f)  \n", msg->joint_names[middle_joint_index].c_str(), middle_joint_index, t,
                       middle_joint_current_velocity, stall_velocity_threshold_grasp_ );

            }

            // If 1st joint of fingers is moving, continue...
            a_joint_is_moving = false;
            if ((index_joint_current_velocity > stall_velocity_threshold_grasp_) || (middle_joint_current_velocity > stall_velocity_threshold_grasp_))
            {
                a_joint_is_moving = true;

                printf("                    index1 or middle1 joints are moving \n");
            }
            else
            {
                printf("                    index1 or middle1 joints are NOT moving \n");
            }
            if (using_sim_) // simulated robot
            {
                // For simulation, if 2nd or 3rd joints are moving, continue...
                // (hard-coded joint indexes, there should be 7 joint velocities in the state msg for simulation)
                if ( (abs(msg->actual.velocities[2]) > stall_velocity_threshold_grasp_)
                        || (abs(msg->actual.velocities[3]) > stall_velocity_threshold_grasp_)
                        || (abs(msg->actual.velocities[5]) > stall_velocity_threshold_grasp_)
                        || (abs(msg->actual.velocities[6]) > stall_velocity_threshold_grasp_) )
                {
                    a_joint_is_moving = true;
                    printf("                    one of 2nd or 3rd digits are moving \n");
                }
                else
                {
                    printf("                   2nd or 2rd digits NOT moving \n");
                }
            }

            if (a_joint_is_moving == true)
            {
                printf("   a_joint_is_moving == true  \n" );

                last_movement_time_ = ros::Time::now();
            }
            // If all fingers joints stalled for certain time, abort Action.
            else if ((ros::Time::now() - last_movement_time_).toSec() > stall_timeout_grasp_)
            {
                // Debug
                printf("\n[reem_hand_grasp_controller] joint has stalled DAVID \n\n");
                printf("   Velocity:  Index finger: %f  Middle finger: %f  \n", index_joint_current_velocity, middle_joint_current_velocity );


                //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *gripper_traj_action_client_;

                //ros::Duration(0.3).sleep();
                //ROS_INFO("Sending cancel goal...");
                //gripper_client_->cancelAllGoals();

                // cancel goal was not working on real robot

                if (using_sim_) // simulated robot
                {
                    // cancel, this works in Sim only
                    gripper_traj_action_client_->cancelGoal();

                    // Send empty trajectory command to stop hand
                    /*
                       control_msgs::FollowJointTrajectoryGoal stop_gripper_traj;
                       stop_gripper_traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_2_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_3_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_2_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_3_joint");
                       gripper_traj_action_client_->sendGoal(stop_gripper_traj);
                    */

                    // Send new trajectory with current positions, to stop hand
                    /*
                       control_msgs::FollowJointTrajectoryGoal stop_gripper_traj;
                       stop_gripper_traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
                       stop_gripper_traj.trajectory.points.resize(1); // 1 single waypoint
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_2_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_3_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_2_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_3_joint");
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[0] ); // thumb
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[1] ); // index1
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[2] );
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[3] );
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[4] ); // middle1
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[5] );
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[6] );
                    stop_gripper_traj.trajectory.points[0].time_from_start = ros::Duration(0.0); // immediately (optional)
                      gripper_traj_action_client_->sendGoal(stop_gripper_traj);
                    */

                }
                else
                {
                    // cancel, this works in Sim only
                    //gripper_traj_action_client_->cancelGoal();

                    // Send empty trajectory command to stop hand
                    // don't use ros::Time::now() in header.stamp, this causes problems
                    control_msgs::FollowJointTrajectoryGoal stop_gripper_traj;
                    stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                    stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                    stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                    gripper_traj_action_client_->sendGoal(stop_gripper_traj);

                    // Send new trajectory with current positions, to stop hand
                    /*
                       control_msgs::FollowJointTrajectoryGoal stop_gripper_traj;
                       stop_gripper_traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
                       stop_gripper_traj.trajectory.points.resize(1); // 1 single waypoint
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
                          stop_gripper_traj.trajectory.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[0] ); // thumb
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[1] ); // index1
                          stop_gripper_traj.trajectory.points[0].positions.push_back( msg->actual.positions[2] );// middle1
                    stop_gripper_traj.trajectory.points[0].time_from_start = ros::Duration(0.0); // immediately
                      gripper_traj_action_client_->sendGoal(stop_gripper_traj);
                    */

                } //endif using_sim_

                active_goal_.setSucceeded(); // goal must be Succeeded, not Aborted, for Manipulation Pipeline to continue after grasping
                has_active_goal_ = false;
            }
            else // joints stopped, but time is below threshold
            {

                printf("   no joints moving, but last movement time (%f sec) < timeout (%f sec)  \n",
                       (ros::Time::now() - last_movement_time_).toSec(), stall_timeout_grasp_ );
            } // endif joint moving
        }
        else
        {
        }

        break;

    // PRE_GRASP state
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP:
        // get index number of thumb joint, from action msg
        thumb_joint_index = find( last_controller_state_->joint_names.begin(), last_controller_state_->joint_names.end(),
                                  "hand_"+left_or_right_+"_thumb_joint" ) - last_controller_state_->joint_names.begin();
        // get goal angle from action msg
        thumb_goal_angle = active_goal_.getGoal()->grasp.pre_grasp_posture.position[thumb_joint_index]; // thumb

        // Debug:
        printf("Controller state for PRE_GRASP:   (position_threshold_pre_grasp_=%f)  \n", position_threshold_pre_grasp_ );
        printf("Joint %s (%d)  pos diff=%f  \n", msg->joint_names[thumb_joint_index].c_str(), thumb_joint_index,
               fabs(msg->actual.positions[thumb_joint_index] - thumb_goal_angle) );

        // If thumb joint is at pre_grasp goal position, the Action is complete
        if (fabs(msg->actual.positions[thumb_joint_index] - thumb_goal_angle) < position_threshold_pre_grasp_)
        {
            // Debug
            printf("[reem_hand_grasp_controller] goal position reached for Thumb \n");

            active_goal_.setSucceeded();
            has_active_goal_ = false;
        }
        else
        {
            last_movement_time_ = ros::Time::now();
        }

        break;

    // RELEASE state
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE:
        // Debug
        printf("Controller state for RELEASE:   (goal threshold_=%f)  (num joints=%d) \n", position_threshold_release_, msg->actual.positions.size()  );

        not_at_goal = false;
        // if all joints are within release threshold position, goal is complete
        for (unsigned int i; i < msg->actual.positions.size(); i++)
        {
            // Debug
            printf("Joint %s (%d)  pos diff=%f  \n", msg->joint_names[i].c_str(), i, fabs(msg->actual.positions[i] - 0.0) );

            if (fabs(msg->actual.positions[i] - 0.0) > position_threshold_release_) not_at_goal = true;
        }
        if (not_at_goal == false)
        {
            // Debug
            printf("[reem_hand_grasp_controller] goal position reached for all joints \n");

            active_goal_.setSucceeded();
            has_active_goal_ = false;
        }
        else
        {
            last_movement_time_ = ros::Time::now();
        }

        break;

    default:
        ROS_ERROR("[reem_hand_grasp_controller]  unknown goal code (%d) in controllerStateCB \n", active_goal_.getGoal()->goal );
        //active_goal_.setAborted();
        return;

    } // end switch
}

// callback for Service the Manipulation Pipeline calls to check if grasp was successful
bool REEMHandGraspController::serviceCallback(object_manipulation_msgs::GraspStatus::Request &request,
        object_manipulation_msgs::GraspStatus::Response &response)
{
    // Assuming the GRASP action finished when the velocity threshold was reached,
    // and the controller is working correctly,
    // we must be grasping the object!
    // Ideally this would call a touch sensor.
    response.is_hand_occupied = true;

    return true;
}

// callback for an Action for testing a touch sensor,
// when the sensor was pressed it cancelled the Action request.
void REEMHandGraspController::callbackTest(const std_msgs::Bool::ConstPtr &msg)
{
    printf("recvd test callback ! \n\n\n");

    // the rest is pasted from cancelCB


    // Debug:
    printf("[reem_hand_grasp_controller]  called cancelCB() \n");

    //  if (active_goal_ == gh)
    // {
    // Stops the controller.
    if (last_controller_state_)
    {
        // Get index number of a specific joint from the controller state message (which might have 2,3or7 joints)
        unsigned int joint_index = find( last_controller_state_->joint_names.begin(), last_controller_state_->joint_names.end(), "hand_"+left_or_right_+"_index_1_joint" )
                                   - last_controller_state_->joint_names.begin();

        // Stop the hand by sending a new traj with the current position
        // don't use ros::Time::now() in header.stamp, this causes problems
        trajectory_msgs::JointTrajectory hand_traj;
        hand_traj.points.resize(1);

        // thumb reads current position & puts that in traj
        hand_traj.joint_names.push_back("hand_"+left_or_right_+"_thumb_joint");
        hand_traj.points[0].positions.push_back( last_controller_state_->actual.positions[0] );

        hand_traj.joint_names.push_back("hand_"+left_or_right_+"_index_1_joint");
        hand_traj.joint_names.push_back("hand_"+left_or_right_+"_middle_1_joint");
        for (unsigned int i=0; i<2; i++)
        {
            hand_traj.points[0].positions.push_back( last_controller_state_->actual.positions[joint_index] ); // goal is current position
        }

        // Set the velocity
        hand_traj.points[0].time_from_start = ros::Duration(0.0); // stop immediately

        pub_controller_command_.publish(hand_traj);

        //printf("[reem_hand_grasp_controller]  stopping at actual.position %f  \n", last_controller_state_->actual.positions[joint_index] );
        printf("[reem_hand_grasp_controller]  stopping...  \n");
        printf("   Position:  Index finger: %f  Middle finger: %f  \n", last_controller_state_->actual.positions[1], last_controller_state_->actual.positions[4] );
        printf("   Velocity:  Index finger: %f  Middle finger: %f  \n", last_controller_state_->actual.velocities[1], last_controller_state_->actual.velocities[4] );
    }

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
    //  }
}

