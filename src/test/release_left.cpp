/*
 * Test the 'release' mechanism of the REEM robot hand.
 *
 * This Action Client connects to reem_hand_grasp_controller, issues a release
 * command and waits until the Action is complete.
 * The release action is 2-phase, opening the fingers first, then thumb.
 *
 * Currently hard-coded for the left hand. 
 * The posture Action Message always has 7 joints, because the hand definition YAML 
 * file can only have one definition. Then reem_hand_grasp_controller just ignores
 * the extra joints when running on the real robot.
 *
 * Initially based on the PR2 'Moving the Gripper' tutorial
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

#include <ros/ros.h>

// SimpleActionClient has the most documentation, and is easy to cancel a goal
#include <actionlib/client/simple_action_client.h>

// Accept Action messages in the format of this posture msg, for compatibility 
// with the ROS Manipulation Pipeline
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>

// These definitions are used in the posture msg
#define PRE_GRASP 1
#define GRASP 2
#define RELEASE 3

//#include <iostream> // cout, for extra debugging

// Action interface
typedef actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> HandActionClient;

class HandTest
{
private:
    HandActionClient* hand_action_client_;

public:
    HandTest()
    {
        // Initialize the client for the Action interface to the hand controller
        // and tell the action client that we want to spin a thread by default
        hand_action_client_ = new HandActionClient("left_hand_controller/grasp_posture_controller", true);

        // Wait for the hand action server to come up
        while(!hand_action_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the 'left_hand_controller/grasp_posture_controller' Action Server to come up");
        }
    }

    ~HandTest()
    {
        delete hand_action_client_;
    }

    // Define PRE_GRASP goal command
    void pre_grasp()
    {
        object_manipulation_msgs::GraspHandPostureExecutionGoal pregrasp;
        pregrasp.goal = PRE_GRASP;
        std::string left_or_right = "left";
        // always 7 joints, then reem_hand_grasp_controller handles the simulated or real robot
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_thumb_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_1_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_2_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_3_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_1_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_2_joint");
        pregrasp.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_3_joint");
        // normally these positions come from hand definition YAML
        pregrasp.grasp.pre_grasp_posture.position.push_back(1.5);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.position.push_back(0.0);
        // effort not used by controller
        pregrasp.grasp.pre_grasp_posture.effort.push_back(100.0); 
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);
        pregrasp.grasp.pre_grasp_posture.effort.push_back(0.0);

        ROS_INFO("Sending pre_grasp goal");
        hand_action_client_->sendGoal(pregrasp);

        hand_action_client_->waitForResult();
        if(hand_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The pre_grasp action is done!");
        }
        else
        {
            ROS_INFO("The pre_grasp action failed.");
            //std::cout << "state is: " << (hand_action_client_->getState()) << " \n";
        }
    }

    // Define GRASP goal command
    void grasp()
    {
        object_manipulation_msgs::GraspHandPostureExecutionGoal grasp;
        grasp.goal = GRASP;
        std::string left_or_right = "left";
        // always 7 joints, then reem_hand_grasp_controller handles the simulated or real robot
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_thumb_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_index_1_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_index_2_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_index_3_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_middle_1_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_middle_2_joint");
        grasp.grasp.grasp_posture.name.push_back("hand_"+left_or_right+"_middle_3_joint");
        // normally these positions come from hand definition YAML.
        // for the simulated robot, reem_hand_grasp_controller moves each finger joint
        // to 4.5 radians in 3 stages, to mimic the behaviour of an underactuated finger.
        grasp.grasp.grasp_posture.position.push_back(1.5); // thumb
        grasp.grasp.grasp_posture.position.push_back(4.5); // fingers
        grasp.grasp.grasp_posture.position.push_back(4.5);
        grasp.grasp.grasp_posture.position.push_back(4.5);
        grasp.grasp.grasp_posture.position.push_back(4.5);
        grasp.grasp.grasp_posture.position.push_back(4.5);
        grasp.grasp.grasp_posture.position.push_back(4.5);
        // effort not used by controller
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);
        grasp.grasp.grasp_posture.effort.push_back(100.0);

        ROS_INFO("Sending grasp goal");
        hand_action_client_->sendGoal(grasp);

        // This code block shows how to cancel the goal, while it's running
        //ros::Duration(0.3).sleep();
        //ROS_INFO("Sending cancel goal...");
        //hand_action_client_->cancelAllGoals();

        hand_action_client_->waitForResult();
        if(hand_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The gripper closed!");
        }
        else
        {
            ROS_INFO("The gripper failed to close.");
            //std::cout << "state is: " << (hand_action_client_->getState()) << " \n";
        }
    }

    // Define RELEASE goal command
    void release()
    {
        object_manipulation_msgs::GraspHandPostureExecutionGoal release;
        release.goal = RELEASE;
        std::string left_or_right = "left";
        // always 7 joints, then reem_hand_grasp_controller handles the simulated or real robot
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_thumb_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_1_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_2_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_index_3_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_1_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_2_joint");
        release.grasp.pre_grasp_posture.name.push_back("hand_"+left_or_right+"_middle_3_joint");
        // normally these positions come from hand definition YAML
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        release.grasp.pre_grasp_posture.position.push_back(0.0);
        // effort not used by controller
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);
        release.grasp.pre_grasp_posture.effort.push_back(0.0);

        ROS_INFO("Sending release goal");
        hand_action_client_->sendGoal(release);

        hand_action_client_->waitForResult();
        if(hand_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The release action is done!");
        }
        else
        {
            ROS_INFO("The release action failed.");
            //std::cout << "state is: " << (hand_action_client_->getState()) << " \n";
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_gripper_test");

    HandTest hand;

    // These methods are blocking, but if you want to test how to cancel a goal, check the 
    // grasp() method for some example code.

    hand.release();
    ros::Duration(1.0).sleep();

    //hand.pre_grasp();
    //ros::Duration(1.5).sleep();

    //hand.grasp();
    //ros::Duration(1.0).sleep();

    //hand.release();

    return 0;
}
