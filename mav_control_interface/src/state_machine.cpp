/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <mav_msgs/default_topics.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>

#include "state_machine.h"

namespace mav_control_interface {

namespace state_machine {

StateMachineDefinition::StateMachineDefinition(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh,
                                               std::shared_ptr<PositionControllerInterface> controller)
    :nh_(nh),
     private_nh_(private_nh),
     controller_(controller)
{
  command_publisher_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

  current_reference_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "command/current_reference", 1);

  state_info_publisher_ = nh_.advertise<std_msgs::String>("state_machine/state_info", 1, true);

  private_nh_.param<bool>("use_rc_teleop", use_rc_teleop_, true);
  private_nh_.param<std::string>("reference_frame", reference_frame_id_, "odom");

  private_nh_.param("thrust_coefficient", thrust_coefficient, thrust_coefficient);
  private_nh_.param("thrust_offset", thrust_offset, thrust_coefficient);
  private_nh_.param("thrust_min",    thrust_min, thrust_min);
  private_nh_.param("thrust_max",    thrust_max, thrust_max);
  ROS_INFO("ROS PARAM thrust_coefficient: %f", thrust_coefficient);
  ROS_INFO("ROS PARAM thrust_offset:      %f", thrust_offset);
  ROS_INFO("ROS PARAM thrust_min:         %f", thrust_min);
  ROS_INFO("ROS PARAM thrust_max:         %f", thrust_max);

  predicted_state_publisher_ = nh_.advertise<visualization_msgs::Marker>( "predicted_state", 0 );
  full_predicted_state_publisher_ = 
    nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>( "full_predicted_state", 1 );
}

void StateMachineDefinition::SetParameters(const Parameters& parameters)
{
  parameters_ = parameters;
}

void StateMachineDefinition::PublishAttitudeCommand (
    const mav_msgs::EigenRollPitchYawrateThrust& command) const
{
    mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);

    mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
    tmp_command.thrust.x() = 0;
    tmp_command.thrust.y() = 0;
    double thz;
    double thz_org = command.thrust.z();
    thz = std::max(0.0, thz_org);
    controler

    thz       = (thz * thrust_coefficient) + thrust_offset; // this transforms it from Force in[N] to thrust in[%]
    ROS_INFO_THROTTLE(0.1, "Throttle org:%f; scaled:%f", thz_org, thz);

    if(thz < thrust_min){
      ROS_WARN_STREAM_THROTTLE(0.1, "Throttle command is below minimum.. set to minimum");
      thz = thrust_min;
    }
    if(thz > thrust_max){
      ROS_WARN_STREAM_THROTTLE(0.1, "Throttle command is too high.. set to max");
      thz = thrust_max;
    }

    tmp_command.thrust.z() = thz;
    msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
    mav_msgs::msgRollPitchYawrateThrustFromEigen(tmp_command, msg.get());
    command_publisher_.publish(msg);
}

void StateMachineDefinition::PublishStateInfo(const std::string& info)
{
  if (state_info_publisher_.getNumSubscribers() > 0) {
    std_msgs::StringPtr msg(new std_msgs::String);
    msg->data = info;
    state_info_publisher_.publish(msg);
  }
}

void StateMachineDefinition::PublishCurrentReference()
{
  ros::Time time_now = ros::Time::now();
  mav_msgs::EigenTrajectoryPoint current_reference;
  controller_->getCurrentReference(&current_reference);

  tf::Quaternion q;
  tf::Vector3 p;
  tf::vectorEigenToTF(current_reference.position_W, p);
  tf::quaternionEigenToTF(current_reference.orientation_W_B, q);

  tf::Transform transform;
  transform.setOrigin(p);
  transform.setRotation(q);

  transform_broadcaster_.sendTransform(
      tf::StampedTransform(transform, time_now, reference_frame_id_, nh_.getNamespace() + "/current_reference"));

  if (current_reference_publisher_.getNumSubscribers() > 0) {
    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_reference, msg.get());
    msg->header.stamp = time_now;
    msg->header.frame_id = reference_frame_id_;
    current_reference_publisher_.publish(msg);
  }
}

void StateMachineDefinition::PublishPredictedState()
{
  if (predicted_state_publisher_.getNumSubscribers() > 0) {
    mav_msgs::EigenTrajectoryPointDeque predicted_state;
    controller_->getPredictedState(&predicted_state);
    visualization_msgs::Marker marker_queue;
    marker_queue.header.frame_id = reference_frame_id_;
    marker_queue.header.stamp = ros::Time();
    marker_queue.type = visualization_msgs::Marker::LINE_STRIP;
    marker_queue.scale.x = 0.05;
    marker_queue.color.a = 1.0;
    marker_queue.color.r = 1.0;

    //    marker_heading.type = visualization_msgs::Marker::ARROW;
    {
      for (size_t i = 0; i < predicted_state.size(); i++) {
        geometry_msgs::Point p;
        p.x = predicted_state.at(i).position_W(0);
        p.y = predicted_state.at(i).position_W(1);
        p.z = predicted_state.at(i).position_W(2);
        marker_queue.points.push_back(p);
      }
    }

    predicted_state_publisher_.publish(marker_queue);
  }

  if (full_predicted_state_publisher_.getNumSubscribers() > 0) {
    mav_msgs::EigenTrajectoryPointDeque predicted_state;
    controller_->getPredictedState(&predicted_state);

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msgMultiDofJointTrajectoryFromEigen(predicted_state, &msg);

    //add in timestamp information
    if (!predicted_state.empty()) {
      msg.header.stamp.fromNSec(predicted_state.front().timestamp_ns -
                         predicted_state.front().time_from_start_ns);
    }

    full_predicted_state_publisher_.publish(msg);
  }
}

} // end namespace state_machine

} // namespace mav_control_interface
