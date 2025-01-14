/*
 * Copyright (c) 2016, Vanderbilt University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Addisu Z. Taddese
 */

#ifndef INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
#define INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_

// Ignition Gazebo headers
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

// Other includes
#include <memory>
#include "storm_gazebo_ros_magnet/dipole_magnet_container.h"

namespace gazebo {

class DipoleMagnet : public ignition::gazebo::System {
 public:
  DipoleMagnet();

  ~DipoleMagnet();

  /// \brief Loads the plugin
  void Load(ignition::gazebo::EntityPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for when subscribers connect
  void Connect();

  /// \brief Callback for when subscribers disconnect
  void Disconnect();

  /// \brief Thread to interact with ROS
  void QueueThread();

  /// \brief Called by the world update start event
  void OnUpdate(const ignition::gazebo::UpdateInfo & /*_info*/);

  /// \brief Publishes data to ros topics
  /// \pram[in] force A vector of force that makes up the wrench to be published
  /// \pram[in] torque A vector of torque that makes up the wrench to be published
  /// \pram[in] mfs A vector of magnetic field data
  void PublishData(
      const ignition::math::Vector3d& force, 
      const ignition::math::Vector3d& torque,
      const ignition::math::Vector3d& mfs);

  /// \brief Calculate force and torque of a magnet on another
  /// \parama[in] p_self Pose of the first magnet
  /// \parama[in] m_self Dipole moment of the first magnet
  /// \parama[in] p_other Pose of the second magnet
  /// \parama[in] m_other Dipole moment of the second magnet on which the force is calculated
  /// \param[out] force Calculated force vector
  /// \param[out] torque Calculated torque vector
  void GetForceTorque(const ignition::math::Pose3d& p_self,  
      const ignition::math::Vector3d& m_self,
      const ignition::math::Pose3d& p_other, 
      const ignition::math::Vector3d& m_other,
      ignition::math::Vector3d& force, 
      ignition::math::Vector3d& torque);

  /// \brief Calculate the magnetic field on all 6 sensors
  /// \parama[in] p_self Pose of the first magnet
  /// \parama[in] p_other Pose of the second magnet
  /// \parama[in] m_other Dipole moment of the second magnet
  /// \param[out] mfs magnetic field sensors
  void GetMFS(const ignition::math::Pose3d& p_self,
      const ignition::math::Pose3d& p_other,
      const ignition::math::Vector3d& m_other,
      ignition::math::Vector3d& mfs);

  // Pointer to the model
 private:
  ignition::gazebo::ModelPtr model;
  ignition::gazebo::LinkPtr link;
  ignition::gazebo::WorldPtr world;

  std::shared_ptr<DipoleMagnetContainer::Magnet> mag;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool should_publish;
  rclcpp::Node::SharedPtr rosnode;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mfs_pub;

  geometry_msgs::msg::WrenchStamped wrench_msg;
  sensor_msgs::msg::MagneticField mfs_msg;

  std::mutex lock;
  int connect_count;

  // Custom Callback Queue
  rclcpp::executors::SingleThreadedExecutor callback_queue;

  ignition::gazebo::common::Time last_time;
  double update_rate;
  // Pointer to the update event connection
  ignition::gazebo::EventPtr update_connection;
};

}  // namespace gazebo

#endif  // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
