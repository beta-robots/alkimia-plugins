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
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

/* Author: Carlos Rosales
   Desc:   Gazebo plugin for the alkimia door, adapted from the gazebo ros control plugin
   using pluginlib
*/

// Boost
#include <boost/bind.hpp>

#include <alkimia_plugins/alkimia_plugins.h>
#include <urdf/model.h>

namespace alkimia_plugins
{

AlkimiaPlugins::~AlkimiaPlugins()
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

// Overloaded Gazebo entry point
void AlkimiaPlugins::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  ROS_INFO_STREAM_NAMED("alkimia_plugins","Loading alkimia_plugins");


  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  // Error message if the model couldn't be found
  if (!parent_model_)
  {
    ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("alkimia_plugins","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get namespace for nodehandle
  if(sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
  }
  else
  {
    robot_namespace_ = parent_model_->GetName(); // default
  }

  // Get robot_description ROS param name
  if (sdf_->HasElement("robotParam"))
  {
    robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
  }
  else
  {
    robot_description_ = "robot_description"; // default
  }

  // Get the robot simulation interface type
  if(sdf_->HasElement("robotSimType"))
  {
    robot_hw_sim_type_str_ = sdf_->Get<std::string>("robotSimType");
  }
  else
  {
    robot_hw_sim_type_str_ = "alkimia_plugins/DefaultDoorSim";
    ROS_DEBUG_STREAM_NAMED("loadThread","Using default plugin for DoorSim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str_<<"\"");
  }

  // Get the robot simulation interface type
  if(sdf_->HasElement("master1"))
  {
    masters_.push_back( sdf_->Get<std::string>("master1") );
  }

  if(sdf_->HasElement("master2"))
  {
    masters_.push_back( sdf_->Get<std::string>("master2") );
  }

  if(sdf_->HasElement("master3"))
  {
    masters_.push_back( sdf_->Get<std::string>("master3") );
  }

  if( masters_.empty() )
  {
    ROS_WARN_STREAM_NAMED("loadThread","Default plugin for DoorSim requires at least one master fin ( " << masters_.size() << " specified in URDF/SDF )\"" <<" The first fin will be used instead\"");
  }
  else
  {
    ROS_INFO_STREAM_NAMED("loadThread","Using " << masters_.size() << " actuated fins.");
  }

  if(sdf_->HasElement("couplingStiffness"))
  {
    coupling_stiffness_ = sdf_->Get<double>("couplingStiffness");
  }
  else
  {
     ROS_WARN_STREAM_NAMED("loadThread","Coupling stiffness not specified, using 100 Nm/rad as default \"");
     coupling_stiffness_ = 100.0;
  }
  ROS_INFO_STREAM_NAMED("loadThread","Using a value of "<< coupling_stiffness_ <<" Nm/rad as coupling stiffness");

  // Get the Gazebo simulation period
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

  // Decide the plugin control period
  if(sdf_->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if( control_period_ < gazebo_period )
    {
      ROS_ERROR_STREAM_NAMED("alkimia_plugins","Desired controller update period ("<<control_period_
        <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else if( control_period_ > gazebo_period )
    {
      ROS_WARN_STREAM_NAMED("alkimia_plugins","Desired controller update period ("<<control_period_
        <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM_NAMED("alkimia_plugins","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
  }

  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(robot_namespace_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (sdf_->HasElement("eStopTopic"))
  {
    const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
    e_stop_sub_ = model_nh_.subscribe(e_stop_topic, 1, &AlkimiaPlugins::eStopCB, this);
  }

  ROS_INFO_NAMED("alkimia_plugins", "Starting alkimia_plugins in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR_NAMED("alkimia_plugins", "Error parsing URDF in alkimia_plugins, plugin not active.\n");
    return;
  }

  // Load the DoorSim abstraction to interface the controllers with the gazebo model
  try
  {
    robot_hw_sim_loader_.reset
      (new pluginlib::ClassLoader<alkimia_plugins::DoorSim>
        ("alkimia_plugins",
          "alkimia_plugins::DoorSim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->initSim(robot_namespace_, model_nh_, parent_model_, urdf_model_ptr, transmissions_, masters_, coupling_stiffness_))
    {
      ROS_FATAL_NAMED("alkimia_plugins","Could not initialize robot simulation interface");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin
      (boost::bind(&AlkimiaPlugins::Update, this));

  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM_NAMED("alkimia_plugins","Failed to create robot simulation interface loader: "<<ex.what());
  }

  ROS_INFO_NAMED("alkimia_plugins", "Loaded alkimia_plugins.");
}

// Called by the world update start event
void AlkimiaPlugins::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  robot_hw_sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    robot_hw_sim_->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_)
    {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else
    {
      if (last_e_stop_active_)
      {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else
      {
        reset_ctrlrs = false;
      }
    }
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  // Update the gazebo model with the result of the controller
  // computation
  robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

// Called on world reset
void AlkimiaPlugins::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

// Get the URDF XML from the parameter server
std::string AlkimiaPlugins::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("alkimia_plugins", "alkimia_plugins is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("alkimia_plugins", "alkimia_plugins is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("alkimia_plugins", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool AlkimiaPlugins::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

// Emergency stop callback
void AlkimiaPlugins::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AlkimiaPlugins);
} // namespace
