// Copyright (c) 2016-2019, Mathias Lüdtke, Samuel Lindgren
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "canopen_chain_node/canopen_chain_component.hpp"

// #include <string>
// #include <memory>
// #include <map>
// #include <vector>
// #include <utility>

// using namespace canopen_chain_node;

namespace canopen_chain_node
{

CanopenChainComponent::CanopenChainComponent()
: LifecycleNode("canopen_chain")
{
  RCLCPP_INFO(this->get_logger(), "Creating canopen_chain component");

  // bus
  declare_parameter("bus.device", rclcpp::ParameterValue("can0"));
  declare_parameter("bus.loopback", rclcpp::ParameterValue("false"));
  declare_parameter("bus.driver_plugin", rclcpp::ParameterValue("socketcan_interface/SocketCANInterface"));
  declare_parameter("bus.master_allocator", rclcpp::ParameterValue("canopen_master/SimpleMasterAllocator"));

  // sync
  declare_parameter("sync.overflow", rclcpp::ParameterValue(10));
  declare_parameter("sync.interval_ms", rclcpp::ParameterValue(0));
  declare_parameter("sync.update_ms", rclcpp::ParameterValue(10));

  // heartbeat
  declare_parameter("heartbeat.msg", rclcpp::ParameterValue("704#05"));
  declare_parameter("heartbeat.rate", rclcpp::ParameterValue(0.0));

  // defaults
  declare_parameter("defaults.eds_pkg", rclcpp::ParameterValue("canopen_chain_node"));
  declare_parameter("defaults.eds_file", rclcpp::ParameterValue("test/eds/ISD860.eds"));

  // nodes
  std::vector<std::string> nodes;
  declare_parameter("canopen_nodes", rclcpp::ParameterValue(std::vector<std::string>(nodes)));

  std::flush(std::cout);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring canopen_chain");

  if (!configure_bus())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (!configure_sync())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (!configure_heartbeat())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (!configure_defaults())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if (!configure_nodes())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool CanopenChainComponent::configure_bus()
{
  RCLCPP_INFO(this->get_logger(), "Configuring bus");

  std::string bus_device;
  get_parameter("bus.device", bus_device);
  RCLCPP_INFO(this->get_logger(), "bus.device: %s", bus_device.c_str());

  bool loopback;
  get_parameter("bus.loopback", loopback);
  RCLCPP_INFO(this->get_logger(), "bus.loopback: %s", loopback ? "true" : "false");

  std::string driver_plugin;
  get_parameter("bus.driver_plugin", driver_plugin);
  RCLCPP_INFO(this->get_logger(), "bus.driver_plugin: %s", driver_plugin.c_str());

  std::string master_allocator;
  get_parameter("bus.master_allocator", master_allocator);
  RCLCPP_INFO(this->get_logger(), "bus.master_allocator: %s", master_allocator.c_str());

  return true;
}

bool CanopenChainComponent::configure_sync()
{
  RCLCPP_INFO(this->get_logger(), "Configuring sync");

  int overflow;
  get_parameter("sync.overflow", overflow);
  RCLCPP_INFO(this->get_logger(), "sync.overflow: %d", overflow);

  int interval_ms;
  get_parameter("sync.interval_ms", interval_ms);
  RCLCPP_INFO(this->get_logger(), "sync.interval_ms: %d", interval_ms);

  int update_ms;
  get_parameter("sync.update_ms", update_ms);
  RCLCPP_INFO(this->get_logger(), "sync.update_ms: %d", update_ms);

  return true;
}

bool CanopenChainComponent::configure_heartbeat()
{
  RCLCPP_INFO(this->get_logger(), "Configuring heartbeat");

  std::string msg;
  get_parameter("heartbeat.msg", msg);
  RCLCPP_INFO(this->get_logger(), "heartbeat.msg: %s", msg.c_str());

  float rate = 0.0;
  get_parameter("heartbeat.rate", rate);
  RCLCPP_INFO(this->get_logger(), "heartbeat.rate: %f", rate);

  return true;
}

bool CanopenChainComponent::configure_defaults()
{
  RCLCPP_INFO(this->get_logger(), "Configuring defaults");

  get_parameter("defaults.eds_pkg", default_eds_pkg_);
  RCLCPP_INFO(this->get_logger(), "defaults.eds_pkg: %s", default_eds_pkg_.c_str());

  get_parameter("defaults.eds_file", default_eds_file_);
  RCLCPP_INFO(this->get_logger(), "defaults.eds_file: %s", default_eds_file_.c_str());

  return true;
}

bool CanopenChainComponent::configure_nodes()
{
  RCLCPP_INFO(this->get_logger(), "Configuring nodes");

  std::vector<std::string> canopen_nodes;
  get_parameter("canopen_nodes", canopen_nodes);

  for (auto &node_name : canopen_nodes)
  {
    int node_id = -1; 
    declare_parameter(node_name + ".id", rclcpp::ParameterValue(node_id));
    get_parameter(node_name + ".id", node_id);
    if (node_id == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "No node_id specified for %s, aborting!", node_name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "%s id: %d", node_name.c_str(), node_id);

    declare_parameter(node_name + ".eds_file", rclcpp::ParameterValue(default_eds_file_));
    std::string eds_file;
    get_parameter(node_name + ".eds_file", eds_file);
    RCLCPP_INFO(this->get_logger(), "%s eds_file: %s", node_name.c_str(), eds_file.c_str());
  }

  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating canopen_chain");
  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating canopen_chain");
  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up canopen_chain");
  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down canopen_chain");
  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Error in canopen_chain");
  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace canopen_chain_node
