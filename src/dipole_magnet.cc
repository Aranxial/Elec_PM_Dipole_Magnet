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

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <iostream>
#include <vector>
#include <cstdint>

#include "storm_gazebo_ros_magnet/dipole_magnet.hpp"

namespace gazebo {

DipoleMagnet::DipoleMagnet(): ModelPlugin() {
  this->connect_count = 0;
}

DipoleMagnet::~DipoleMagnet() {
  event::Events::DisconnectWorldUpdateBegin(this->update_connection);
  if (this->should_publish) {
    this->queue.clear();
    this->queue.disable();
    this->rosnode->shutdown();
    this->callback_queue_thread.join();
  }
  if (this->mag) {
    DipoleMagnetContainer::Get().Remove(this->mag);
  }
}

void DipoleMagnet::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  gzdbg << "Loading DipoleMagnet plugin" << std::endl;

  this->mag = std::make_shared<DipoleMagnetContainer::Magnet>();

  // load parameters
  this->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName")) {
    gzerr << "DipoleMagnet plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  } else {
    this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  this->link = this->model->GetLink(this->link_name);
  if (!this->link) {
    gzerr << "Error: link named " << this->link_name << " does not exist" << std::endl;
    return;
  }

  this->should_publish = false;
  if (_sdf->HasElement("shouldPublish")) {
    this->should_publish = _sdf->GetElement("shouldPublish")->Get<bool>();
  }

  if (!_sdf->HasElement("updateRate")) {
    gzmsg << "DipoleMagnet plugin missing <updateRate>, defaults to 0.0"
          " (as fast as possible)" << std::endl;
    this->update_rate = 0;
  } else {
    this->update_rate = _sdf->GetElement("updateRate")->Get<double>();
  }

  if (_sdf->HasElement("calculate")) {
    this->mag->calculate = _sdf->Get<bool>("calculate");
  } else {
    this->mag->calculate = true;
  }

  if (_sdf->HasElement("dipole_moment")) {
    this->mag->moment = _sdf->Get<ignition::math::Vector3d>("dipole_moment");
  }

  if (_sdf->HasElement("xyzOffset")) {
    this->mag->offset.Pos() = _sdf->Get<ignition::math::Vector3d>("xyzOffset");
  }

  if (_sdf->HasElement("rpyOffset")) {
    ignition::math::Vector3d rpy_offset = _sdf->Get<ignition::math::Vector3d>("rpyOffset");
    this->mag->offset.Rot() = ignition::math::Quaterniond(rpy_offset);
  }

  if (this->should_publish) {
    if (!_sdf->HasElement("topicNs")) {
      gzmsg << "DipoleMagnet plugin missing <topicNs>,"
            " will publish on namespace " << this->link_name << std::endl;
    } else {
      this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();
    }

    if (!rclcpp::is_initialized()) {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
            "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
            "the gazebo_ros package. If you want to use this plugin without ROS, "
            "set <shouldPublish> to false" << std::endl;
      return;
    }

    // ROS 2 Node Initialization
    this->rosnode = rclcpp::Node::make_shared("dipole_magnet_plugin");
    this->wrench_pub = rosnode->create_publisher<geometry_msgs::msg::WrenchStamped>(
        this->topic_ns + "/wrench", 1);
    this->mfs_pub = rosnode->create_publisher<sensor_msgs::msg::MagneticField>(
        this->topic_ns + "/mfs", 1);
  }

  this->mag->model_id = this->model->GetId();

  gzmsg << "Loaded Gazebo dipole magnet plugin on " << this->model->GetName() << std::endl;

  DipoleMagnetContainer::Get().Add(this->mag);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DipoleMagnet::OnUpdate, this, _1));
}

void DipoleMagnet::OnUpdate(const common::UpdateInfo & /*_info*/) {
  // Calculate the force from all other magnets
  ignition::math::Pose3d p_self = this->link->WorldPose();
  p_self.Pos() += -p_self.Rot().RotateVector(this->mag->offset.Pos());
  p_self.Rot() *= this->mag->offset.Rot().Inverse();

  this->mag->pose = p_self;

  if (!this->mag->calculate)
    return;

  DipoleMagnetContainer& dp = DipoleMagnetContainer::Get();

  ignition::math::Vector3d moment_world = p_self.Rot().RotateVector(this->mag->moment);

  ignition::math::Vector3d force(0, 0, 0);
  ignition::math::Vector3d torque(0, 0, 0);
  ignition::math::Vector3d mfs(0, 0, 0);
  for (DipoleMagnetContainer::MagnetPtrV::iterator it = dp.magnets.begin(); it < dp.magnets.end(); it++) {
    std::shared_ptr<DipoleMagnetContainer::Magnet> mag_other = *it;
    if (mag_other->model_id != this->mag->model_id) {
      ignition::math::Pose3d p_other = mag_other->pose;
      ignition::math::Vector3d m_other = p_other.Rot().RotateVector(mag_other->moment);

      ignition::math::Vector3d force_tmp;
      ignition::math::Vector3d torque_tmp;
      GetForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);

      force += force_tmp;
      torque += torque_tmp;

      ignition::math::Vector3d mfs_tmp;
      GetMFS(p_self, p_other, m_other, mfs_tmp);

      mfs += mfs_tmp;

      this->link->AddForce(force_tmp);
      this->link->AddTorque(torque_tmp);
    }
  }

  this->PublishData(force, torque, mfs);
}

void DipoleMagnet::PublishData(
    const ignition::math::Vector3d& force,
    const ignition::math::Vector3d& torque,
    const ignition::math::Vector3d& mfs) {
  if (this->should_publish) {
    // Rate control and ROS 2 message publishing
    auto cur_time = this->world->SimTime();
    if (this->update_rate > 0 &&
        (cur_time - this->last_time).Double() < (1.0 / this->update_rate)) {
      return;
    }

    // Publish wrench and magnetic field data
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.frame_id = "world";
    wrench_msg.header.stamp.sec = cur_time.sec;
    wrench_msg.header.stamp.nanosec = cur_time.nsec;

    wrench_msg.wrench.force.x = force.X();
    wrench_msg.wrench.force.y = force.Y();
    wrench_msg.wrench.force.z = force.Z();
    wrench_msg.wrench.torque.x = torque.X();
    wrench_msg.wrench.torque.y = torque.Y();
    wrench_msg.wrench.torque.z = torque.Z();

    sensor_msgs::msg::MagneticField mfs_msg;
    mfs_msg.header.frame_id = this->link_name;
    mfs_msg.header.stamp.sec = cur_time.sec;
    mfs_msg.header.stamp.nanosec = cur_time.nsec;

    mfs_msg.magnetic_field.x = mfs.X();
    mfs_msg.magnetic_field.y = mfs.Y();
    mfs_msg.magnetic_field.z = mfs.Z();

    this->wrench_pub->publish(wrench_msg);
    this->mfs_pub->publish(mfs_msg);
  }
}

void DipoleMagnet::GetForceTorque(const ignition::math::Pose3d& p_self,
                                  const ignition::math::Vector3d& m_self,
                                  const ignition::math::Pose3d& p_other,
                                  const ignition::math::Vector3d& m_other,
                                  ignition::math::Vector3d& force,
                                  ignition::math::Vector3d& torque) {
  ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
  ignition::math::Vector3d p_unit = p / p.Length();

  ignition::math::Vector3d m1 = m_other;
  ignition::math::Vector3d m2 = m_self;

  double K = 3.0 * 1e-7 / std::pow(p.Length(), 4);
  force = K * (m2 * (m1.Dot(p_unit)) + m1 * (m2.Dot(p_unit)) +
               p_unit * (m1.Dot(m2)) - 5 * p_unit * (m1.Dot(p_unit)) * (m2.Dot(p_unit)));

  double Ktorque = 1e-7 / std::pow(p.Length(), 3);
  ignition::math::Vector3d B1 = Ktorque * (3 * (m1.Dot(p_unit)) * p_unit - m1);
  torque = m2.Cross(B1);
}

void DipoleMagnet::GetMFS(const ignition::math::Pose3d& p_self,
                          const ignition::math::Pose3d& p_other,
                          const ignition::math::Vector3d& m_other,
                          ignition::math::Vector3d& mfs) {
  ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
  ignition::math::Vector3d p_unit = p / p.Length();

  double K = 3.0 * 1e-7 / std::pow(p.Length(), 4);
  mfs = K * (3 * m_other.Dot(p_unit) * p_unit - m_other);
}

GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet)

}  // namespace gazebo
