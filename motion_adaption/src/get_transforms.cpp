/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2011, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Marcus Liebhardt */


#include "motion_adaption/motion_adaption.h"


bool MotionAdaption::getTransforms()
{
    /*
    std::cout<<"world_ref_frame_str_ "<<world_ref_frame_str_<<std::endl;
    std::cout<<"user_torso_str_ "<<user_torso_str_<<std::endl;
    std::cout<<"user_head_str_ "<<user_head_str_<<std::endl;
    std::cout<<"user_r_shoulder_str_ "<<user_r_shoulder_str_<<std::endl;
    std::cout<<"user_r_elbow_str_ "<<user_r_elbow_str_<<std::endl;
    std::cout<<"user_r_hand_str_ "<<user_r_hand_str_<<std::endl;
    std::cout<<"user_l_shoulder_str_ "<<user_l_shoulder_str_<<std::endl;
    std::cout<<"user_l_elbow_str_ "<<user_l_elbow_str_<<std::endl;
    std::cout<<"user_l_hand_str_ "<<user_l_hand_str_<<std::endl;
    */
    try
    {

        //std::cout<<"0"<<std::endl;

        //torso
        tf_listener_.waitForTransform(world_ref_frame_str_, user_torso_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(world_ref_frame_str_, user_torso_str_, ros::Time(0), tf_usr_torso_);

        //std::cout<<"1"<<std::endl;

        //head
        tf_listener_.waitForTransform(user_torso_str_, user_head_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_head_str_, ros::Time(0), tf_usr_head_);

        //std::cout<<"2"<<std::endl;

        // right shoulder
        tf_listener_.waitForTransform(user_torso_str_, user_r_shoulder_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_r_shoulder_str_, ros::Time(0), tf_usr_r_shoulder_);

        //std::cout<<"3"<<std::endl;

        // right shoulder to right elbow
        tf_listener_.waitForTransform(user_r_shoulder_str_, user_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_r_shoulder_str_, user_r_elbow_str_, ros::Time(0), tf_usr_r_shoulder_elbow_);

        //std::cout<<"4"<<std::endl;

        // right elbow to right hand
        tf_listener_.waitForTransform(user_r_elbow_str_, user_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_r_elbow_str_, user_r_hand_str_, ros::Time(0), tf_usr_r_elbow_hand_);

        //std::cout<<"5"<<std::endl;

        // right elbow
        tf_listener_.waitForTransform(user_torso_str_, user_r_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_r_elbow_str_, ros::Time(0), tf_usr_r_elbow_);

        //std::cout<<"6"<<std::endl;

        // right hand
        tf_listener_.waitForTransform(user_torso_str_, user_r_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_r_hand_str_, ros::Time(0), tf_usr_r_hand_);

        //std::cout<<"7"<<std::endl;

        // left shoulder
        tf_listener_.waitForTransform(user_torso_str_, user_l_shoulder_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_l_shoulder_str_, ros::Time(0), tf_usr_l_shoulder_);

        //std::cout<<"8"<<std::endl;

        // left shoulder to left elbow
        tf_listener_.waitForTransform(user_l_shoulder_str_, user_l_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_l_shoulder_str_, user_l_elbow_str_, ros::Time(0), tf_usr_l_shoulder_elbow_);

        //std::cout<<"9"<<std::endl;

        // left elbow to left hand
        tf_listener_.waitForTransform(user_l_elbow_str_, user_l_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_l_elbow_str_, user_l_hand_str_, ros::Time(0), tf_usr_l_elbow_hand_);

        //std::cout<<"10"<<std::endl;

        // left elbow
        tf_listener_.waitForTransform(user_torso_str_, user_l_elbow_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_l_elbow_str_, ros::Time(0), tf_usr_l_elbow_);

        //std::cout<<"11"<<std::endl;

        // left hand
        tf_listener_.waitForTransform(user_torso_str_, user_l_hand_str_, ros::Time(0), ros::Duration(wait_for_tf_));
        tf_listener_.lookupTransform(user_torso_str_, user_l_hand_str_, ros::Time(0), tf_usr_l_hand_);

        //std::cout<<"12"<<std::endl;

    }
    catch (tf::TransformException const &ex)
    {
        ROS_DEBUG("%s",ex.what());
        ROS_WARN("(Step 1) Couldn't get one or more transformations of the user.");
        ROS_WARN("No further processing will be done!");
        return false;
    }
    return true;
}

