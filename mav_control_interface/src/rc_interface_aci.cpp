/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* Copyright (c) 2014, Sammy Omari, ASL, ETH Zurich, Switzerland
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

#include <mav_control_interface/rc_interface_aci.h>

namespace mav_control_interface {

RcInterfaceAci::RcInterfaceAci(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : RcInterfaceBase(),
      nh_(nh),
      private_nh_(private_nh),
      is_on_(false),
      axes_ch_index_mode(5),
      axes_ch_index_control(6),
      axes_ch_index_rc_on(6),
      axes_ch_index_wheel(4),
      axes_ch_index_right_horiz(0),
      axes_ch_index_right_vert(1),
      axes_ch_index_left_horiz(3),
      axes_ch_index_left_vert(2),

      axes_ch_factor_mode(-1),
      axes_ch_factor_rc_on(1),
      axes_ch_factor_control(1),
      axes_ch_factor_wheel(-1),

      axes_ch_factor_right_horiz(-1.0),
      axes_ch_factor_right_vert(-1.0),
      axes_ch_factor_left_horiz(-1.0),
      axes_ch_factor_left_vert(-1.0)
{
      rc_sub_ = nh_.subscribe("rc", 1, &RcInterfaceAci::rcCallback, this);
      private_nh_.param("axes_ch_index_mode",axes_ch_index_mode, axes_ch_index_mode);
      private_nh_.param("axes_ch_index_rc_on",axes_ch_index_rc_on, axes_ch_index_rc_on);
      private_nh_.param("axes_ch_index_control",axes_ch_index_control, axes_ch_index_control);
      private_nh_.param("axes_ch_index_wheel",axes_ch_index_wheel, axes_ch_index_wheel);

      private_nh_.param("axes_ch_index_right_horiz", axes_ch_index_right_horiz,  axes_ch_index_right_horiz);
      private_nh_.param("axes_ch_index_right_vert",  axes_ch_index_right_vert,   axes_ch_index_right_vert);
      private_nh_.param("axes_ch_index_left_horiz",  axes_ch_index_left_horiz,   axes_ch_index_left_horiz);
      private_nh_.param("axes_ch_index_left_vert",   axes_ch_index_left_vert,    axes_ch_index_left_vert);

      private_nh_.param("axes_ch_factor_mode",        axes_ch_factor_mode,         axes_ch_factor_mode);
      private_nh_.param("axes_ch_factor_rc_on",       axes_ch_factor_rc_on,        axes_ch_factor_rc_on);
      private_nh_.param("axes_ch_factor_control",     axes_ch_factor_control,      axes_ch_factor_control);
      private_nh_.param("axes_ch_factor_wheel",       axes_ch_factor_wheel,        axes_ch_factor_wheel);

      private_nh_.param("axes_ch_factor_right_horiz", axes_ch_factor_right_horiz,  axes_ch_factor_right_horiz);
      private_nh_.param("axes_ch_factor_right_vert",  axes_ch_factor_right_vert,   axes_ch_factor_right_vert);
      private_nh_.param("axes_ch_factor_left_horiz",  axes_ch_factor_left_horiz,   axes_ch_factor_left_horiz);
      private_nh_.param("axes_ch_factor_left_vert",   axes_ch_factor_left_vert,    axes_ch_factor_left_vert);
}

void RcInterfaceAci::rcCallback(const sensor_msgs::JoyConstPtr& msg)
{
  is_on_ = isRcOn(msg);
  last_data_.timestamp = msg->header.stamp;

  if (is_on_) {
    last_data_.right_side =    msg->axes[axes_ch_index_right_horiz] * axes_ch_factor_right_horiz;
    last_data_.right_up_down = msg->axes[axes_ch_index_right_vert]  * axes_ch_factor_right_vert;

    last_data_.left_side =    msg->axes[axes_ch_index_left_horiz]   * axes_ch_factor_left_horiz;
    last_data_.left_up_down = msg->axes[axes_ch_index_left_vert]    * axes_ch_factor_left_vert;

    RcData::ControlInterface old_control = last_data_.control_interface;
    if ((msg->axes[axes_ch_index_control] * axes_ch_factor_control) > 0.0)
      last_data_.control_interface = RcData::ControlInterface::ON;
    else
      last_data_.control_interface = RcData::ControlInterface::OFF;

    if(old_control != last_data_.control_interface)
    {
        if(last_data_.control_interface == RcData::ControlInterface::ON)
        {
            ROS_INFO("RC INTERFACE is ON");
        }
        else
        {
            ROS_INFO("RC INTERFACE is OFF");
        }
    }

    RcData::ControlMode old_mode = last_data_.control_mode;
    double mode_val = msg->axes[axes_ch_index_mode] * axes_ch_factor_mode;
    if (mode_val <= -0.5)
      last_data_.control_mode = RcData::ControlMode::MANUAL;
    else if (mode_val > -0.5 && mode_val < 0.5)
      last_data_.control_mode = RcData::ControlMode::ALTITUDE_CONTROL;
    else
      last_data_.control_mode = RcData::ControlMode::POSITION_CONTROL;

    if(old_mode != last_data_.control_mode)
    {
        switch (last_data_.control_mode)
        {
        case RcData::ControlMode::MANUAL:
            ROS_INFO("Control Mode canged to MANUAL");
            break;
        case RcData::ControlMode::ALTITUDE_CONTROL:
            ROS_INFO("Control Mode canged to ALTITUDE_CONTROL");
            break;
        case RcData::ControlMode::POSITION_CONTROL:
            ROS_INFO("Control Mode canged to POSITION_CONTROL");
            break;
        default:
            ROS_ERROR("Control Mode canged to Unknown value!!!");
            break;
        }

    }
    last_data_.wheel = msg->axes[axes_ch_index_wheel] * axes_ch_factor_wheel;
  }
  else {  //set to zero if RC is off
    ROS_WARN_STREAM_THROTTLE(5.0, "Detected RC Off.");
    last_data_.right_up_down = 0.0;
    last_data_.right_side = 0.0;
    last_data_.left_up_down = -1.0;
    last_data_.left_side = 0.0;
    last_data_.control_interface = RcData::ControlInterface::OFF;
    last_data_.control_mode = RcData::ControlMode::MANUAL;
    last_data_.wheel = 0.0;
  }
  this->updated();
}

std::string RcInterfaceAci::getName() const
{
  return std::string("ACI rc interface");
}

RcData RcInterfaceAci::getRcData() const
{
  return last_data_;
}

bool RcInterfaceAci::isActive() const
{
  if (!is_on_)
    return false;
  else if (std::abs(last_data_.right_up_down) > STICK_DEADZONE
      || std::abs(last_data_.right_side) > STICK_DEADZONE
      || std::abs(last_data_.left_up_down) > STICK_DEADZONE
      || std::abs(last_data_.left_side) > STICK_DEADZONE) {
    return true;
  }
  return false;
}

bool RcInterfaceAci::isOn() const
{
  return is_on_;
}

float RcInterfaceAci::getStickDeadzone() const
{
  return STICK_DEADZONE;
}

bool RcInterfaceAci::isRcOn(const sensor_msgs::JoyConstPtr& msg) const
{
  return (msg->axes[axes_ch_index_rc_on] * axes_ch_factor_rc_on > 0.5);
}

}  // end namespace mav_control_interface
