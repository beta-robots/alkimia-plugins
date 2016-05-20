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

#include <alkimia_plugins/default_door_sim.h>


namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace alkimia_plugins
{


bool DefaultDoorSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions,
  std::vector<std::string> masters,
  double coupling_stiffness)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  coupling_stiffness_  = coupling_stiffness;

  // Resize vectors to our DOFs
  n_dof_ = transmissions.size();
  n_adof_ = masters.size();
  n_pdof_ = (n_dof_ - 2*masters.size()) / 2;
  n_bdof_ = transmissions.size() / 2;

  ROS_INFO_STREAM_NAMED("default_door_sim","Number of joints '" << n_dof_);
  ROS_INFO_STREAM_NAMED("default_door_sim","Active joints '" << n_adof_);
  ROS_INFO_STREAM_NAMED("default_door_sim","Passive joints '" << n_pdof_);
  ROS_INFO_STREAM_NAMED("default_door_sim","Bottom joints '" << n_bdof_);

  // active with motor
  active_top_joint_names_.resize( n_adof_ );
  active_top_joint_types_.resize( n_adof_ );
  active_top_joint_lower_limits_.resize( n_adof_ );
  active_top_joint_upper_limits_.resize( n_adof_ );
  active_top_joint_effort_limits_.resize( n_adof_ );
  active_top_joint_control_methods_.resize( n_adof_ );
  active_top_joint_position_.resize( n_adof_ );
  active_top_joint_velocity_.resize( n_adof_ );
  active_top_joint_effort_.resize( n_adof_ );
  active_top_joint_effort_command_.resize( n_adof_ );

  // passive with spring
  passive_top_joint_names_.resize( n_pdof_ );
  passive_top_joint_types_.resize( n_pdof_ );
  passive_top_joint_lower_limits_.resize( n_pdof_ );
  passive_top_joint_upper_limits_.resize( n_pdof_ );
  passive_top_joint_effort_limits_.resize( n_pdof_ );
  passive_top_joint_control_methods_.resize( n_pdof_ );
  passive_top_pid_controllers_.resize( n_pdof_ );
  passive_top_joint_position_.resize( n_pdof_ );
  passive_top_joint_velocity_.resize( n_pdof_ );
  passive_top_joint_effort_.resize( n_pdof_ );
  passive_top_joint_effort_command_.resize( n_pdof_ );

  // coupled with belt
  bottom_joint_names_.resize( n_bdof_ );
  bottom_joint_types_.resize( n_bdof_ );
  bottom_joint_lower_limits_.resize( n_bdof_ );
  bottom_joint_upper_limits_.resize( n_bdof_ );
  bottom_joint_effort_limits_.resize( n_bdof_ );
  bottom_joint_control_methods_.resize( n_bdof_ );
  bottom_pid_controllers_.resize( n_bdof_ );
  bottom_joint_position_.resize( n_bdof_ );
  bottom_joint_velocity_.resize( n_bdof_ );
  bottom_joint_effort_.resize( n_bdof_ );
  bottom_joint_position_command_.resize( n_bdof_ );
  bottom_last_joint_position_command_.resize( n_bdof_ );

  // validate all
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("default_door_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_door_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("default_door_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("default_door_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_door_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
      //continue;
    }
  }

  // Initialize active values
  for(unsigned int j=0; j < n_adof_; j++)
  {

    // Add data from transmission
    active_top_joint_names_[j] = masters.at(j) + std::string("_top_joint");
    active_top_joint_position_[j] = 1.0;
    active_top_joint_velocity_[j] = 0.0;
    active_top_joint_effort_[j] = 1.0;  // N/m for continuous joints
    active_top_joint_effort_command_[j] = 0.0;
    // Debug
    ROS_INFO_STREAM_NAMED("default_door_sim","Loading active joint '" << active_top_joint_names_[j]
      << "' of type EffortJointInterface'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        active_top_joint_names_[j], &active_top_joint_position_[j], &active_top_joint_velocity_[j], &active_top_joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    // Create effort joint interface
    active_top_joint_control_methods_[j] = EFFORT;
    hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(active_top_joint_names_[j]), &active_top_joint_effort_command_[j]);
    ej_interface_.registerHandle(joint_handle);

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_door_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(active_top_joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << active_top_joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    registerJointLimits(active_top_joint_names_[j], joint_handle, active_top_joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &active_top_joint_types_[j], &active_top_joint_lower_limits_[j], &active_top_joint_upper_limits_[j],
                        &active_top_joint_effort_limits_[j]);
  }

  // passive counter
  unsigned int p = 0;
  // Initialize passive values
  for(unsigned int j=0; j < n_dof_; j++)
  {

    // find only passive top joints
    std::string passive_joint_name = transmissions[j].joints_[0].name_;
    bool is_active = false;
    bool is_belted = false;
    for( unsigned int a=0; a < n_adof_; a++)
    {
        if( passive_joint_name.compare( active_top_joint_names_.at(a) ) == 0 )
                is_active = true;
        if( passive_joint_name.find(std::string("bottom")) !=std::string::npos )
                is_belted = true;
    }
    if( is_active || is_belted)
            continue;

    // Add data from transmission
    passive_top_joint_names_[p] = passive_joint_name;
    passive_top_joint_position_[p] = 1.0;
    passive_top_joint_velocity_[p] = 0.0;
    passive_top_joint_effort_[p] = 1.0;  // N/m for continuous joints
    passive_top_joint_effort_command_[p] = 0.0;

    // Debug
    ROS_INFO_STREAM_NAMED("default_door_sim","Loading passive joint '" << passive_top_joint_names_[p]
      << "' of type EffortJointInterface'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        passive_top_joint_names_[p], &passive_top_joint_position_[p], &passive_top_joint_velocity_[p], &passive_top_joint_effort_[p]));

    passive_top_joint_control_methods_[p] = EFFORT;

    hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(passive_top_joint_names_[p]), &passive_top_joint_effort_command_[p]);

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_door_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(passive_top_joint_names_[p]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << passive_top_joint_names_[p]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    registerJointLimits(passive_top_joint_names_[p], joint_handle, passive_top_joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &passive_top_joint_types_[p], &passive_top_joint_lower_limits_[p], &passive_top_joint_upper_limits_[p],
                        &passive_top_joint_effort_limits_[p]);

    p++;
  }

  // belted counter
  unsigned int b = 0;
  // Initialize belted values
  for(unsigned int j=0; j < n_dof_; j++)
  {

    // find only passive top joints
    std::string belted_joint_name = transmissions[j].joints_[0].name_;
    if( std::string::npos != belted_joint_name.find(std::string("top")))
              continue;

    // Add data from transmission
    bottom_joint_names_[b] = belted_joint_name;
    bottom_joint_position_[b] = 1.0;
    bottom_joint_velocity_[b] = 0.0;
    bottom_joint_effort_[b] = 1.0;  // N/m for continuous joints
    bottom_joint_position_command_[b] = 0.0;

    // Debug
    ROS_INFO_STREAM_NAMED("default_door_sim","Loading joint '" << bottom_joint_names_[b]
      << "' of type PositionJointInterface'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        bottom_joint_names_[b], &bottom_joint_position_[b], &bottom_joint_velocity_[b], &bottom_joint_effort_[b]));

    bottom_joint_control_methods_[b] = POSITION_PID;

    hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(bottom_joint_names_[b]), &bottom_joint_position_command_[b]);

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_door_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(bottom_joint_names_[b]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << bottom_joint_names_[b]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    // Initialize the PID controller
    const ros::NodeHandle nh(model_nh, "/alkimia_door/pid_gains/" +
                               bottom_joint_names_[b]);
    bottom_pid_controllers_[b].init(nh, true);

    registerJointLimits(bottom_joint_names_[b], joint_handle, bottom_joint_control_methods_[b],
                        joint_limit_nh, urdf_model,
                        &bottom_joint_types_[b], &bottom_joint_lower_limits_[b], &bottom_joint_upper_limits_[b],
                        &bottom_joint_effort_limits_[b]);

    b++;

  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void DefaultDoorSim::readSim(ros::Time time, ros::Duration period)
{
  // read active
  for(unsigned int j=0; j < n_adof_; j++)
  {
    active_top_joint_position_[j] += angles::shortest_angular_distance(active_top_joint_position_[j],
                            sim_joints_[j]->GetAngle(0).Radian());
    active_top_joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    active_top_joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  // read passive
  for(unsigned int j=0; j < n_pdof_; j++)
  {
    passive_top_joint_position_[j] += angles::shortest_angular_distance(passive_top_joint_position_[j],
                            sim_joints_[n_adof_ + j]->GetAngle(0).Radian());
    passive_top_joint_velocity_[j] = sim_joints_[n_adof_ + j]->GetVelocity(0);
    passive_top_joint_effort_[j] = sim_joints_[n_adof_ + j]->GetForce((unsigned int)(0));
  }

  // read bottom
  for(unsigned int j=0; j < n_bdof_; j++)
  {
    bottom_joint_position_[j] += angles::shortest_angular_distance(bottom_joint_position_[j],
                            sim_joints_[n_adof_ + n_pdof_ + j]->GetAngle(0).Radian());
    bottom_joint_velocity_[j] = sim_joints_[n_adof_ + n_pdof_ + j]->GetVelocity(0);
    bottom_joint_effort_[j] = sim_joints_[n_adof_ + n_pdof_ + j]->GetForce((unsigned int)(0));
  }
}

void DefaultDoorSim::writeSim(ros::Time time, ros::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  /*if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }
  */

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);

  for(unsigned int j=0; j < n_adof_; j++)
  {
     const double effort = active_top_joint_effort_command_[j];
     sim_joints_[j]->SetForce(0, effort);
  }

  for(unsigned int j=0; j < n_pdof_; j++)
  {
      // Compute the coupling effort command
     // ATTENTION: this work with only one master fin so far !
     double angular_displacement;
     if( j == 0 )
     {
        angles::shortest_angular_distance_with_limits(passive_top_joint_position_[j],
                                                      active_top_joint_position_[0],
                                                      passive_top_joint_lower_limits_[j],
                                                      passive_top_joint_upper_limits_[j],
                                                      angular_displacement);
     }
     else
     {
        angles::shortest_angular_distance_with_limits(passive_top_joint_position_[j],
                                                      passive_top_joint_position_[j-1],
                                                      passive_top_joint_lower_limits_[j],
                                                      passive_top_joint_upper_limits_[j],
                                                      angular_displacement);
     }

     // compute spring effort and add damping
     passive_top_joint_effort_command_[j] = coupling_stiffness_ * angular_displacement - 0.15*coupling_stiffness_*passive_top_joint_velocity_[j];

     const double effort = passive_top_joint_effort_command_[j];
     sim_joints_[n_adof_ + j]->SetForce(0, effort);
  }

  for(unsigned int j=0; j < n_bdof_; j++)
  {
      // Compute the belted position command
      // ATTENTION: this depends on the ordering above for parsing active and passive joints, and admits
      //            only one fin master !!
      double error;
      if( j == 0 )
              bottom_joint_position_command_[j] = 2*active_top_joint_position_[0];
      else
              bottom_joint_position_command_[j] = 2*passive_top_joint_position_[j-1];

      // control the belt coupling
      /*angles::shortest_angular_distance_with_limits(bottom_joint_position_[j],
                                                    bottom_joint_position_command_[j],
                                                    bottom_joint_lower_limits_[j],
                                                    bottom_joint_upper_limits_[j],
                                                    error);

      const double effort_limit = bottom_joint_effort_limits_[j];
      const double effort = clamp(bottom_pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);
      sim_joints_[n_adof_ + n_pdof_ + j]->SetForce(0, effort);*/

      sim_joints_[n_adof_ + n_pdof_ + j]->SetAngle(0, bottom_joint_position_command_[j]);
  }
}

void DefaultDoorSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void DefaultDoorSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(alkimia_plugins::DefaultDoorSim, alkimia_plugins::DoorSim)
