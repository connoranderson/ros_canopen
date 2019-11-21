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

#ifndef CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_COMPONENT_HPP_
#define CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_COMPONENT_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "socketcan_interface/interface.hpp"

// #include <memory>
// #include <vector>
// #include <string>
// #include <utility>
// #include <map>
// #include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"

namespace canopen_chain_node
{

class CanopenChainComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CanopenChainComponent();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(rclcpp_lifecycle::State const &);

private:
    std::string default_eds_pkg_;
    std::string default_eds_file_;

    can::DriverInterfaceSharedPtr interface_;

    bool configure_bus();
    bool configure_sync();
    bool configure_heartbeat();
    bool configure_defaults();
    bool configure_nodes();
};

}  // namespace canopen_chain_node

#endif  // CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_COMPONENT_HPP_
