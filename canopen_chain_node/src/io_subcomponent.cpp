// Copyright (c) 2019, Samuel Lindgren
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

#include "canopen_chain_node/io_subcomponent.hpp"

using namespace std::chrono_literals;

namespace canopen_chain_node
{

IOSubcomponent::IOSubcomponent(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services_interface,
    std::string canopen_node_name,
    std::shared_ptr<canopen::Node> canopen_node)
{
    base_interface_ = base_interface;
    timers_interface_ = timers_interface;

    timer_callback_group_ =  base_interface->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(logging_interface->get_logger(), 
            "Creating IO Profile [401] Subcomponenet for %s",
            canopen_node_name.c_str());

    auto timer_callback = [this, logging_interface]() -> void {
        RCLCPP_INFO(logging_interface->get_logger(), "Hello, world!");
    };
    
    timer_ = create_wall_timer(2s, timer_callback, timer_callback_group_);
}

} // namespace canopen_chain_node