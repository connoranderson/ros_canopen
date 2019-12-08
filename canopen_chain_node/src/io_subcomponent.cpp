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
    canopen::ObjectStorageSharedPtr canopen_object_storage): 
    base_interface_(base_interface),
    logging_interface_(logging_interface),
    timers_interface_(timers_interface),
    topics_interface_(topics_interface),
    services_interface_(services_interface),
    canopen_node_name_(canopen_node_name),
    canopen_object_storage_(canopen_object_storage)
{
    RCLCPP_INFO(logging_interface_->get_logger(), 
            "Creating IO Profile [401] Subcomponenet for %s",
            canopen_node_name_.c_str());

    canopen::ObjectStorage::Entry<uint8_t> number_of_digital_input_bytes;
    canopen_object_storage_->entry(number_of_digital_input_bytes, 0x6000, 0);
    RCLCPP_INFO(logging_interface_->get_logger(), 
            "Number of digital input bytes %d",
            number_of_digital_input_bytes.get_cached());

    digital_input_bytes_.clear();
    for(int i=1; i <= number_of_digital_input_bytes.get_cached(); i++)
    {
        auto digital_input_byte = std::make_shared<canopen::ObjectStorage::Entry<uint8_t>>();
        canopen_object_storage_->entry(*digital_input_byte.get(), 0x6000, i);
        digital_input_bytes_.push_back(digital_input_byte);
        RCLCPP_INFO(logging_interface_->get_logger(), 
                "Adding digital input byte %d", i);
    }
}

void from_byte(uint8_t byte, bool bool_array[8])
{
    for (int i=0; i < 8; ++i)
        bool_array[i] = (byte & (1<<i)) != 0;
}

void IOSubcomponent::activate()
{

    // device_inputs_publisher_ = topics_interface_->create_publisher<canopen_msgs::msg::DeviceInputs>("/inputs", 10);

    timer_callback_group_ =  base_interface_->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

   auto timer_callback = [this]() -> void {
        auto inputs_msg = canopen_msgs::msg::DeviceInputs();

        for (auto const& digital_input_byte: digital_input_bytes_)
        {
            bool bool_array[8];
            from_byte(digital_input_byte->get(), bool_array);
            for (auto const& input_state: bool_array) {
                inputs_msg.digital_inputs.push_back(input_state);
            }
        }

        device_inputs_publisher_->publish(inputs_msg);
    };
    
    timer_ = create_wall_timer(1s, timer_callback, timer_callback_group_);
}

void IOSubcomponent::deactivate()
{
    RCLCPP_INFO(logging_interface_->get_logger(), "Deactivating IO Subcomponnet for %s",
                canopen_node_name_.c_str());
    timer_->cancel();
}

} // namespace canopen_chain_node