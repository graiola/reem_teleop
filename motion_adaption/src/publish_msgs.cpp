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


void MotionAdaption::publishData()
{

  // Torso
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/torso_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_torso_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_torso_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_torso_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_torso_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_torso_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_torso_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_torso_goal_.getRotation().w();

  pub_torso_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("torso_goal",10);

  pub_torso_pose_.publish(transfStamp_);

  // Head
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/head_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_head_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_head_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_head_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_head_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_head_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_head_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_head_goal_.getRotation().w();

  pub_head_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("head_goal",10);

  pub_head_pose_.publish(transfStamp_);

  // Right elbow
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/r_elbow_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_r_elbow_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_r_elbow_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_r_elbow_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_r_elbow_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_r_elbow_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_r_elbow_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_r_elbow_goal_.getRotation().w();

  pub_r_elbow_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("r_elbow_goal",10);

  pub_r_elbow_pose_.publish(transfStamp_);

  // Right hand
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/r_hand_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_r_hand_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_r_hand_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_r_hand_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_r_hand_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_r_hand_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_r_hand_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_r_hand_goal_.getRotation().w();

  pub_r_hand_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("r_hand_goal",10);

  pub_r_hand_pose_.publish(transfStamp_);

  // Left elbow
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/l_elbow_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_l_elbow_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_l_elbow_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_l_elbow_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_l_elbow_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_l_elbow_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_l_elbow_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_l_elbow_goal_.getRotation().w();

  pub_l_elbow_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("l_elbow_goal",10);

  pub_l_elbow_pose_.publish(transfStamp_);

  // Left hand
  transfStamp_.header.stamp = ros::Time::now();
  //transfStamp_.header.frame_id = "/l_hand_adapted";
  transfStamp_.header.frame_id = ref_frame;
  transfStamp_.transform.translation.x = tf_l_hand_goal_.getOrigin().x();
  transfStamp_.transform.translation.y = tf_l_hand_goal_.getOrigin().y();
  transfStamp_.transform.translation.z = tf_l_hand_goal_.getOrigin().z();
  transfStamp_.transform.rotation.x = tf_l_hand_goal_.getRotation().x();
  transfStamp_.transform.rotation.y = tf_l_hand_goal_.getRotation().y();
  transfStamp_.transform.rotation.z = tf_l_hand_goal_.getRotation().z();
  transfStamp_.transform.rotation.w = tf_l_hand_goal_.getRotation().w();

  pub_l_hand_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("l_hand_goal",10);

  pub_l_hand_pose_.publish(transfStamp_);

}

