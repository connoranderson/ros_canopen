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

#ifndef CANOPEN_CHAIN_NODE__IO_SUBCOMPONENT_HPP_
#define CANOPEN_CHAIN_NODE__IO_SUBCOMPONENT_HPP_

#include <canopen_master/canopen.hpp>
#include <rclcpp/rclcpp.hpp>

namespace canopen_chain_node
{

class IOSubcomponent
{
public:
    IOSubcomponent(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services_interface,
        std::string canopen_node_name,
        std::shared_ptr<canopen::Node> canopen_node);
private:
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface_;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface_;
    
    std::shared_ptr<rclcpp::callback_group::CallbackGroup> timer_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    // NOTE(sam): create_wall_timer is not available througt timers_interface
    // for some reason
    template<typename DurationRepT, typename DurationT, typename CallbackT>
    typename rclcpp::WallTimer<CallbackT>::SharedPtr
    create_wall_timer(
        std::chrono::duration<DurationRepT, DurationT> period,
        CallbackT callback,
        rclcpp::callback_group::CallbackGroup::SharedPtr group)
    {
        auto timer = rclcpp::WallTimer<CallbackT>::make_shared(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::move(callback), this->base_interface_->get_context());
        timers_interface_->add_timer(timer, group);
        return timer;
    }

};

}  // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__IO_SUBCOMPONENT_HPP_