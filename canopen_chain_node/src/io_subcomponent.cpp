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
    rclcpp_lifecycle::LifecycleNode *parent_component,
    std::string canopen_node_name,
    canopen::ObjectStorageSharedPtr canopen_object_storage): 
    parent_component_(parent_component),
    canopen_node_name_(canopen_node_name),
    canopen_object_storage_(canopen_object_storage)
{
    RCLCPP_INFO(parent_component_->get_logger(), 
            "Creating IO Profile [401] Subcomponenet for %s",
            canopen_node_name_.c_str());

    // FIXME(sam): try to catch all possible timeout exceptions when reading/writing
    // to object dictionary to prevent program termination in case the canopen node stops responding.

    configure_digital_outputs();
    configure_digital_inputs();
}

void IOSubcomponent::configure_digital_outputs()
{
    canopen::ObjectStorage::Entry<uint8_t> number_of_digital_output_bytes;
    canopen_object_storage_->entry(number_of_digital_output_bytes, 0x6200, 0);

    digital_output_bytes_.clear();
    for(int i=1; i <= number_of_digital_output_bytes.get_cached(); i++)
    {
        auto digital_output_byte = std::make_shared<canopen::ObjectStorage::Entry<uint8_t>>();
        canopen_object_storage_->entry(*digital_output_byte.get(), 0x6200, i);
        digital_output_bytes_.push_back(digital_output_byte);
    }

    std::vector<std::string> digital_output_names;
    if (!parent_component_->get_parameter(canopen_node_name_ + ".io.digital_output_names", digital_output_names)) {
        parent_component_->declare_parameter(canopen_node_name_ + ".io.digital_output_names", 
                                             rclcpp::ParameterValue(digital_output_names));
        parent_component_->get_parameter(canopen_node_name_ + ".io.digital_output_names", digital_output_names);
    }

    digital_output_names_.clear();

    for (auto const& digital_output_name: digital_output_names)
    {
        digital_output_names_.push_back(digital_output_name);
    }

    while (digital_output_names_.size() < digital_output_bytes_.size() * 8)
    {
        digital_output_names_.push_back("");
    }

    // Use QoS settings to emulate ROS1 latched topic
    device_outputs_publisher_ = parent_component_->create_publisher<canopen_msgs::msg::DeviceOutputs>(
        canopen_node_name_ + "/outputs", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    
    last_outputs_message = canopen_msgs::msg::DeviceOutputs();
}

void IOSubcomponent::configure_digital_inputs()
{
    canopen::ObjectStorage::Entry<uint8_t> number_of_digital_input_bytes;
    canopen_object_storage_->entry(number_of_digital_input_bytes, 0x6000, 0);

    digital_input_bytes_.clear();
    for(int i=1; i <= number_of_digital_input_bytes.get_cached(); i++)
    {
        auto digital_input_byte = std::make_shared<canopen::ObjectStorage::Entry<uint8_t>>();
        canopen_object_storage_->entry(*digital_input_byte.get(), 0x6000, i);
        digital_input_bytes_.push_back(digital_input_byte);
    }

    std::vector<std::string> digital_input_names;
    if (!parent_component_->get_parameter(canopen_node_name_ + ".io.digital_input_names", digital_input_names)) {
        parent_component_->declare_parameter(canopen_node_name_ + ".io.digital_input_names", rclcpp::ParameterValue(digital_input_names));
        parent_component_->get_parameter(canopen_node_name_ + ".io.digital_input_names", digital_input_names);
    }

    digital_input_names_.clear();

    for (auto const& digital_input_name: digital_input_names)
    {
        digital_input_names_.push_back(digital_input_name);
    }

    while (digital_input_names_.size() < digital_input_bytes_.size() * 8)
    {
        digital_input_names_.push_back("");
    }

    // Use QoS settings to emulate ROS1 latched topic
    device_inputs_publisher_ = parent_component_->create_publisher<canopen_msgs::msg::DeviceInputs>(
        canopen_node_name_ + "/inputs", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    last_inputs_message = canopen_msgs::msg::DeviceInputs();
}

void IOSubcomponent::handle_set_digital_output(
        const std::shared_ptr<canopen_msgs::srv::SetDigitalOutput::Request> request,
        std::shared_ptr<canopen_msgs::srv::SetDigitalOutput::Response> response)
{
    if (request->output_name == "")
    {
        int output_index = request->output_index;
        RCLCPP_INFO(parent_component_->get_logger(), 
                "Setting digital output with index %d", output_index);
        if(set_digital_output(output_index, request->output_on))        
        {
            response->success = true;
            response->message = "Set digital output using index";
        } else
        {
            response->success = false;
            response->message = "Digital output index is out of available range.";
        }

    } else
    {
        RCLCPP_INFO(parent_component_->get_logger(), 
                "Setting digital outputs with name: %s", request->output_name.c_str());
        int number_of_outputs_set = 0;

        auto iter = digital_output_names_.begin();
        while ((iter = std::find(iter, digital_output_names_.end(), request->output_name)) != digital_output_names_.end())
        {
            int output_index = iter - digital_output_names_.begin();
            
            if(!set_digital_output(output_index, request->output_on))        
            {
                response->success = false;
                response->message = "Something went wrong setting digital output by name";
                return;
            }

            number_of_outputs_set++;
            iter++;
        }

        if (number_of_outputs_set > 0)
        {
            response->success = false;
            response->message = "Digital output not found by name!";
        } else
        {
            response->success = true;
            response->message = std::to_string(number_of_outputs_set) + " outputs set";
        }
    }
}

bool IOSubcomponent::set_digital_output(int output_index, bool output_on)
{
    int output_byte_index = output_index / 8;
    int output_bit_index = output_index % 8;

    try
    {
        auto digital_output_byte_entry = digital_output_bytes_.at(output_byte_index);
        uint8_t digital_output_byte = digital_output_byte_entry->get_cached();

        if (output_on)
        {
            digital_output_byte |=  1UL << output_bit_index; // set bit
        } else {
            digital_output_byte &=  ~(1UL << output_bit_index); // clear bit
        }

        digital_output_byte_entry->set(digital_output_byte);

    } catch (const std::out_of_range & ex)
    {
        RCLCPP_WARN(parent_component_->get_logger(), 
                "Digital output with index %d is out of range", 
                output_index);
        return false;
    }

    return true;
}

void IOSubcomponent::activate()
{
    device_outputs_publisher_->on_activate();
    device_inputs_publisher_->on_activate();

    set_object_srv_ = parent_component_->create_service<canopen_msgs::srv::SetDigitalOutput>(
        "set_digital_output", 
        std::bind(&IOSubcomponent::handle_set_digital_output, this,
        std::placeholders::_1, std::placeholders::_2));

    auto timer_callback = [this]() -> void {
        publish_digital_outputs();
        publish_digital_inputs();
    };

    timer_ = parent_component_->create_wall_timer(100ms, timer_callback);

    // FIXME(sam): seems like latching is does not work for ros2-web-bridge or ros2 topic echo atm...
    auto extra_timer_callback = [this]() -> void {
        last_inputs_message = canopen_msgs::msg::DeviceInputs();
        last_outputs_message = canopen_msgs::msg::DeviceOutputs();
        publish_digital_outputs();
        publish_digital_inputs();
    };

    lathing_not_working_for_web_ui_temporary_timer_ = parent_component_->create_wall_timer(2s, extra_timer_callback);
}

void from_byte(uint8_t byte, bool bool_array[8])
{
    for (int i=0; i < 8; ++i)
        bool_array[i] = (byte & (1<<i)) != 0;
}

void IOSubcomponent::publish_digital_outputs()
{
    auto outputs_msg = canopen_msgs::msg::DeviceOutputs();
    outputs_msg.digital_output_names = digital_output_names_;

    for (auto const& digital_output_byte: digital_output_bytes_)
    {
        bool bool_array[8];
        // NOTE(sam): initializing seems to set all outputs to 0,
        // so the cache should always be in sync when connected?
        from_byte(digital_output_byte->get_cached(), bool_array);
        for (auto const& output_state: bool_array) {
            outputs_msg.digital_outputs.push_back(output_state);
        }
    }

    if (outputs_msg != last_outputs_message)
    {
        device_outputs_publisher_->publish(outputs_msg);
        last_outputs_message = outputs_msg;
    }
}

void IOSubcomponent::publish_digital_inputs()
{
    auto inputs_msg = canopen_msgs::msg::DeviceInputs();
    inputs_msg.digital_input_names = digital_input_names_;

    for (auto const& digital_input_byte: digital_input_bytes_)
    {
        bool bool_array[8];
        from_byte(digital_input_byte->get(), bool_array);
        for (auto const& input_state: bool_array) {
            inputs_msg.digital_inputs.push_back(input_state);
        }
    }

    if (inputs_msg != last_inputs_message)
    {
        device_inputs_publisher_->publish(inputs_msg);
        last_inputs_message = inputs_msg;
    }
}

void IOSubcomponent::deactivate()
{
    RCLCPP_INFO(parent_component_->get_logger(), "Deactivating IO Subcomponnet for %s",
                canopen_node_name_.c_str());
    timer_->cancel();
    lathing_not_working_for_web_ui_temporary_timer_->cancel();

    set_object_srv_.reset();
    device_outputs_publisher_->on_deactivate();
    device_inputs_publisher_->on_deactivate();
}

} // namespace canopen_chain_node