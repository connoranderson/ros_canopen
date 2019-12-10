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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <canopen_msgs/msg/device_inputs.hpp>
#include <canopen_msgs/msg/device_outputs.hpp>
#include <canopen_msgs/srv/set_digital_output.hpp>

namespace canopen_chain_node
{

class IOSubcomponent
{
public:
    IOSubcomponent(
        rclcpp_lifecycle::LifecycleNode *parent_component,
        std::string canopen_node_name,
        canopen::ObjectStorageSharedPtr canopen_object_storage);

    void configure_digital_outputs();
    void configure_digital_inputs();

    void activate();
    void deactivate();

    void publish_digital_outputs();
    void publish_digital_inputs();

private:
    rclcpp_lifecycle::LifecycleNode *parent_component_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr lathing_not_working_for_web_ui_temporary_timer_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<canopen_msgs::msg::DeviceInputs>> device_inputs_publisher_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<canopen_msgs::msg::DeviceOutputs>> device_outputs_publisher_;

    rclcpp::Service<canopen_msgs::srv::SetDigitalOutput>::SharedPtr set_object_srv_;

    std::string canopen_node_name_;
    canopen::ObjectStorageSharedPtr canopen_object_storage_;

    std::vector<std::string> digital_input_names_;
    std::vector<std::string> digital_output_names_;

    std::vector<std::shared_ptr<canopen::ObjectStorage::Entry<uint8_t>>> digital_input_bytes_;
    std::vector<std::shared_ptr<canopen::ObjectStorage::Entry<uint8_t>>> digital_output_bytes_;

    canopen_msgs::msg::DeviceOutputs last_outputs_message;
    canopen_msgs::msg::DeviceInputs last_inputs_message;
    
    void handle_set_digital_output(
            const std::shared_ptr<canopen_msgs::srv::SetDigitalOutput::Request> request,
            std::shared_ptr<canopen_msgs::srv::SetDigitalOutput::Response> response);
    
    bool set_digital_output(int output_index, bool output_on);

};

}  // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__IO_SUBCOMPONENT_HPP_