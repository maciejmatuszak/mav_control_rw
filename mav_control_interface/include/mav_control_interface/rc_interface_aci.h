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

#ifndef RC_INTERFACE_ACI_H_
#define RC_INTERFACE_ACI_H_

#include <mav_control_interface/rc_interface.h>
#include <nav_msgs/Odometry.h>

namespace mav_control_interface {

class RcInterfaceAci : public RcInterfaceBase {
 public:
  static constexpr float STICK_DEADZONE = 0.1;

  RcInterfaceAci(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  virtual std::string getName() const;
  virtual bool isActive() const;
  virtual bool isOn() const;
  virtual RcData getRcData() const;
  virtual float getStickDeadzone() const;

 private:
  void rcCallback(const sensor_msgs::JoyConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  bool isRcOn(const sensor_msgs::JoyConstPtr& msg) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber rc_sub_;
  ros::Subscriber odometry_syb_;


  ros::ServiceClient wp_nav_srv_client_;
  std::string lock_tf_frame_id_;
  bool lock_next_odometry_;

  RcData last_data_;
  bool is_on_;
  int axes_ch_index_mode;
  int axes_ch_index_rc_on;
  int axes_ch_index_control;
  int axes_ch_index_lock_tf;

  int axes_ch_index_right_vert;
  int axes_ch_index_right_horiz;
  int axes_ch_index_left_vert;
  int axes_ch_index_left_horiz;

  double axes_ch_factor_right_vert;
  double axes_ch_factor_right_horiz;
  double axes_ch_factor_left_vert;
  double axes_ch_factor_left_horiz;
  double axes_ch_factor_mode;
  double axes_ch_factor_rc_on;
  double axes_ch_factor_control;
  double axes_ch_factor_lock_tf;

};
}  // end namespace mav_control_interface
#endif /* RC_INTERFACE_H_ */
