/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

/* Author: Carlos Rosales
   Desc:   Gazebo plugin for the alkimia door, adapted from the gazebo ros control plugin
   using pluginlib
*/

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_DOOR_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_DOOR_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>


#include <alkimia_plugins/door_sim.h>

namespace alkimia_plugins
{

class DefaultDoorSim : public alkimia_plugins::DoorSim
{
public:

  virtual bool initSim(const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions,
    std::vector<std::string> masters,
    double coupling_stiffness);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;
  unsigned int n_adof_;
  unsigned int n_pdof_;
  unsigned int n_bdof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;

  std::vector<std::string> active_top_joint_names_;
  std::vector<int> active_top_joint_types_;
  std::vector<double> active_top_joint_lower_limits_;
  std::vector<double> active_top_joint_upper_limits_;
  std::vector<double> active_top_joint_effort_limits_;
  std::vector<ControlMethod> active_top_joint_control_methods_;
  std::vector<double> active_top_joint_position_;
  std::vector<double> active_top_joint_velocity_;
  std::vector<double> active_top_joint_effort_;
  std::vector<double> active_top_joint_effort_command_;

  std::vector<std::string> passive_top_joint_names_;
  std::vector<int> passive_top_joint_types_;
  std::vector<double> passive_top_joint_lower_limits_;
  std::vector<double> passive_top_joint_upper_limits_;
  std::vector<double> passive_top_joint_effort_limits_;
  std::vector<ControlMethod> passive_top_joint_control_methods_;
  std::vector<control_toolbox::Pid> passive_top_pid_controllers_;
  std::vector<double> passive_top_joint_position_;
  std::vector<double> passive_top_joint_velocity_;
  std::vector<double> passive_top_joint_effort_;
  std::vector<double> passive_top_joint_effort_command_;

  std::vector<std::string> bottom_joint_names_;
  std::vector<int> bottom_joint_types_;
  std::vector<double> bottom_joint_lower_limits_;
  std::vector<double> bottom_joint_upper_limits_;
  std::vector<double> bottom_joint_effort_limits_;
  std::vector<ControlMethod> bottom_joint_control_methods_;
  std::vector<control_toolbox::Pid> bottom_pid_controllers_;
  std::vector<double> bottom_joint_position_;
  std::vector<double> bottom_joint_velocity_;
  std::vector<double> bottom_joint_effort_;
  std::vector<double> bottom_joint_position_command_;
  std::vector<double> bottom_last_joint_position_command_;

  double coupling_stiffness_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
};

typedef boost::shared_ptr<DefaultDoorSim> DefaultDoorSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_DEFAULT_DOOR_SIM_H_
