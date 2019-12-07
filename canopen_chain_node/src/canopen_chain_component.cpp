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

#include <string>
#include <experimental/filesystem>
#include <memory>
// #include <map>
#include <vector>
// #include <utility>

// using namespace canopen_chain_node;

using namespace std::chrono_literals;

namespace canopen_chain_node
{

CanopenChainComponent::CanopenChainComponent()
: LifecycleNode("canopen_chain"), LayerStack("ROS Stack"),
  driver_loader_("socketcan_interface", "can::DriverInterface"),
  master_allocator_("canopen_master", "canopen::Master::Allocator"),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(this->get_logger(), "Creating canopen_chain component");

  std::string hardware_id = "Chain Node";
  diagnostic_updater_.setHardwareID(hardware_id);
  diagnostic_updater_.add("canopen main", this, &CanopenChainComponent::report_diagnostics);

  // bus
  declare_parameter("bus.device", rclcpp::ParameterValue("can0"));
  declare_parameter("bus.loopback", rclcpp::ParameterValue("false"));
  declare_parameter("bus.driver_plugin",
    rclcpp::ParameterValue("socketcan_interface/SocketCANInterface"));
  declare_parameter("bus.master_allocator",
    rclcpp::ParameterValue("canopen_master/SimpleMasterAllocator"));

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

void CanopenChainComponent::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // TODO(sam): add mutex
  canopen::LayerReport report;
  if (canopen::Layer::getLayerState() == canopen::Layer::Off) 
  {
    stat.summary(stat.WARN, "Not initialized");
  } else
  {
    diag(report);
    if (report.bounded<canopen::LayerStatus::Unbounded>())
    {
      // NOTE(sam): I think this runs if the  report is "valid"...
      stat.summary(report.get(), report.reason());
      for (std::vector<std::pair<std::string, std::string>>::const_iterator it =
           report.values().begin(); it != report.values().end(); ++it)
      {
        stat.add(it->first, it->second);
      }
    }
  }
  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring canopen_chain");

  srv_list_object_dictionaries_ = create_service<canopen_msgs::srv::ListObjectDictionaries>(
    "list_object_dictionaries",
    std::bind(&CanopenChainComponent::handle_list_objet_dictionaries, this, 
    std::placeholders::_1, std::placeholders::_2));

  srv_get_object_ = create_service<canopen_msgs::srv::GetObject>(
    "get_object",
    std::bind(&CanopenChainComponent::handle_get_object, this, 
    std::placeholders::_1, std::placeholders::_2));

  srv_set_object_ = create_service<canopen_msgs::srv::SetObject>(
    "set_object",
    std::bind(&CanopenChainComponent::handle_set_object, this, 
    std::placeholders::_1, std::placeholders::_2));

  if (configure_bus() && configure_sync() && configure_heartbeat() && 
      configure_defaults() && configure_nodes())
  {
    std::flush(std::cout);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } else
  {
    std::flush(std::cout);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

}

void CanopenChainComponent::handle_list_objet_dictionaries(
    const std::shared_ptr<canopen_msgs::srv::ListObjectDictionaries::Request> request,
    std::shared_ptr<canopen_msgs::srv::ListObjectDictionaries::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Listing object dictionaries!");

  for (auto node_name: request->nodes) {
    RCLCPP_INFO(this->get_logger(), "Listing object dictionary for node %s", node_name.c_str());

    auto object_dict_msg = canopen_msgs::msg::ObjectDictionary();
    object_dict_msg.node = node_name;

     std::map<std::string, canopen::NodeSharedPtr>::iterator it =
      nodes_lookup_.find(node_name);
      if (it == nodes_lookup_.end()) {
        RCLCPP_WARN(this->get_logger(), "Node %s not found, can't list object dictionary");
      } else {
        try {
          canopen::ObjectDict::ObjectDictMap::const_iterator entry_it;
          while(it->second->getStorage()->dict_->iterate(entry_it))
          {
            auto object_description_msg = canopen_msgs::msg::ObjectDescription();
            object_description_msg.index = std::string(entry_it->first);
            object_description_msg.parameter_name = std::string(entry_it->second->desc);
            object_description_msg.data_type = entry_it->second->data_type;
            object_description_msg.readable = entry_it->second->readable;
            object_description_msg.writable = entry_it->second->writable;

            object_dict_msg.object_descriptions.push_back(object_description_msg);
          }
        } catch (std::exception &e) {
          RCLCPP_WARN(this->get_logger(), boost::diagnostic_information(e));
        }
      }

    response->object_dictionaries.push_back(object_dict_msg);
  }

  std::flush(std::cout);
}

void CanopenChainComponent::handle_get_object(
    const std::shared_ptr<canopen_msgs::srv::GetObject::Request> request,
    std::shared_ptr<canopen_msgs::srv::GetObject::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Getting object: %s from node: %s", 
              request->object.c_str(), request->node.c_str());

  if (!request->cached && getLayerState() == Off)
  {
    std::string error_message = "Reading data from device is not possible when main layer is 'Off'";
    RCLCPP_WARN(this->get_logger(), error_message);
    response->message = error_message;
    response->success = false;
    return;
  }

  auto node_iterator = nodes_lookup_.find(request->node);
  if (node_iterator == nodes_lookup_.end()) {
    RCLCPP_WARN(this->get_logger(), "Node %s not found, can't get object!");
    response->message = "node not found";
    response->success = false;
  } else {
    try {
      std::string value = node_iterator->second->getStorage()->getStringReader(
        canopen::ObjectDict::Key(request->object), request->cached)();
      // NOTE(sam): the \x symbol is not properly handled in ros2-web-bridge, 
      // removing for now to make the web-ui work
      response->value = value;

      response->message = "Got object without issues!";
      response->success = true;
      
    } catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), 
                  "Exception while trying to read from object dictionary for %s: %s",
                  request->node.c_str(),
                  boost::diagnostic_information(e).c_str());
      response->message = boost::diagnostic_information(e);
    }
  }
  
}

void CanopenChainComponent::handle_set_object(
      const std::shared_ptr<canopen_msgs::srv::SetObject::Request> request,
      std::shared_ptr<canopen_msgs::srv::SetObject::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Setting object: %s from node: %s to: %s", 
              request->object.c_str(), request->node.c_str(), request->value.c_str());


  if (!request->cached && getLayerState() == Off)
  {
    std::string error_message = "Writing data to device is not possible when main layer is 'Off'";
    RCLCPP_WARN(this->get_logger(), error_message);
    response->message = error_message;
    response->success = false;
    return;
  }

  auto node_iterator = nodes_lookup_.find(request->node);
  if (node_iterator == nodes_lookup_.end()) {
    response->message = "node not found";
  } else {
    try {
      node_iterator->second->getStorage()->getStringWriter(
        canopen::ObjectDict::Key(request->object), 
        request->cached)(request->value);
        response->success = true;
    } catch (std::exception &e) {
      RCLCPP_WARN(this->get_logger(), 
                  "Exception while trying to write to object dictionary for %s: %s",
                  request->node.c_str(),
                  boost::diagnostic_information(e).c_str());
      response->message = boost::diagnostic_information(e);
    }
  }
 
}


bool CanopenChainComponent::configure_bus()
{
  RCLCPP_INFO(this->get_logger(), "Configuring bus");

  std::string can_device;
  std::string driver_plugin;
  std::string master_allocator;
  bool loopback;

  get_parameter("bus.device", can_device);
  RCLCPP_INFO(this->get_logger(), "bus.device: %s", can_device.c_str());

  get_parameter("bus.driver_plugin", driver_plugin);
  RCLCPP_INFO(this->get_logger(), "bus.driver_plugin: %s", driver_plugin.c_str());

  get_parameter("bus.master_allocator", master_allocator);
  RCLCPP_INFO(this->get_logger(), "bus.master_allocator: %s", master_allocator.c_str());

  get_parameter("bus.loopback", loopback);
  RCLCPP_INFO(this->get_logger(), "bus.loopback: %s", loopback ? "true" : "false");

  try {
    interface_ = driver_loader_.createInstance(driver_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), ex.what());
    return false;
  }

  // TODO(sam): create state listener
  try {
    master_ = master_allocator_.allocateInstance(master_allocator, can_device, interface_);
  } catch (const std::exception &e)
  {
    // TODO(sam): get boost diagnostic_information
    // std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), "Got execption while loading master allocator!");
    return false;
  }

  if (!master_) {
    RCLCPP_ERROR(this->get_logger(), "Could not allocate master");
  }

  add(std::make_shared<canopen::CANLayer>(interface_, can_device, loopback));

  return true;
}

bool CanopenChainComponent::configure_sync()
{
  RCLCPP_INFO(this->get_logger(), "Configuring sync");

  int sync_overflow;
  get_parameter("sync.overflow", sync_overflow);
  RCLCPP_INFO(this->get_logger(), "sync.overflow: %d", sync_overflow);

  int sync_interval_ms;
  get_parameter("sync.interval_ms", sync_interval_ms);
  RCLCPP_INFO(this->get_logger(), "sync.interval_ms: %d", sync_interval_ms);

  int update_ms;
  get_parameter("sync.update_ms", update_ms);
  RCLCPP_INFO(this->get_logger(), "sync.update_ms: %d", update_ms);

  update_period_ms_ = update_ms; 

  // sync_ = master_->getSync(
  //   canopen::SyncProperties(can::MsgHeader(0x80), sync_interval_ms, sync_overflow));

  // if (!sync_ && sync_interval_ms) {
  //   RCLCPP_ERROR(this->get_logger(), "Initializing sync master failed");
  //   return false;
  // }

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
  nodes_.reset(new canopen::LayerGroupNoDiag<canopen::Node>("301 layer"));

  emcy_handlers_.reset(
    new canopen::LayerGroupNoDiag<canopen::EMCYHandler>("EMCY layer"));

  std::vector<std::string> canopen_nodes;
  get_parameter("canopen_nodes", canopen_nodes);

  for (auto & node_name : canopen_nodes) {
    if(!configure_node(node_name))
    {
      return false;
    }
  }
  
  add(nodes_);
  add(emcy_handlers_);

  return true;
}

bool CanopenChainComponent::configure_node(std::string node_name)
{
  // Read parameters
  int node_id = -1;
  if (!get_parameter(node_name + ".id", node_id)) {
    declare_parameter(node_name + ".id", rclcpp::ParameterValue(node_id));
    get_parameter(node_name + ".id", node_id);
  }

  if (node_id == -1) {
    RCLCPP_ERROR(this->get_logger(), "No node_id specified for %s, aborting!", node_name.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "%s id: %d", node_name.c_str(), node_id);

  std::string eds_file;
  if (!get_parameter(node_name + ".eds_file", eds_file)) {
    declare_parameter(node_name + ".eds_file", rclcpp::ParameterValue(default_eds_file_));
    get_parameter(node_name + ".eds_file", eds_file);
  }
  RCLCPP_INFO(this->get_logger(), "%s eds_file: %s", node_name.c_str(), eds_file.c_str());

  std::string eds_pkg;
  if (!get_parameter(node_name + ".eds_pkg", eds_pkg)) {
    declare_parameter(node_name + ".eds_pkg", rclcpp::ParameterValue(default_eds_pkg_));
    get_parameter(node_name + ".eds_pkg", eds_pkg);
  }
  RCLCPP_INFO(this->get_logger(), "%s eds_pkg: %s", node_name.c_str(), eds_pkg.c_str());

  // Find and parse Electronic Data Sheet (EDS)
  std::string eds_pkg_share_directory = "";
  std::string eds_full_path = eds_file;

  if (!eds_pkg.empty()) {
    try {
      eds_pkg_share_directory =
          ament_index_cpp::get_package_share_directory(eds_pkg);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "eds_pkg share directory not found!");
      return false;
    }

    eds_full_path =
        (std::experimental::filesystem::path(eds_pkg_share_directory) / eds_file)
            .make_preferred()
            .native();
  }
  RCLCPP_DEBUG(this->get_logger(), node_name + " eds full path: %s",
              eds_full_path.c_str());

  auto exists = [this](const std::string &name) -> bool {
      struct stat buffer;
      return stat(name.c_str(), &buffer) == 0;
  };

  if (!exists(eds_full_path)) {
    RCLCPP_ERROR(this->get_logger(),
                  node_name + " eds file: %s does not exist!",
                  eds_full_path.c_str());
    return false;
  }

  canopen::ObjectDict::Overlay canopen_object_dict_overlay;
  canopen::ObjectDictSharedPtr canopen_object_dictionary;
  // TODO(sam): parse overlay from params
  try {
     canopen_object_dictionary = 
      canopen::ObjectDict::fromFile(eds_full_path, canopen_object_dict_overlay);
    if (!canopen_object_dictionary) {
      RCLCPP_ERROR(this->get_logger(),
                  "EDS '" + eds_file + "' could not be parsed!");
      return false;
    }
  } catch (std::runtime_error& re) {
    RCLCPP_ERROR(this->get_logger(),
                 "Runtime error while parsing '" + eds_file + "': " + re.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
                 "Exception while parsing '" + eds_file + "'!");
    return false;
  }

  // TODO(sam): Use sync argument when creating node
  canopen::NodeSharedPtr node =
    std::make_shared<canopen::Node>(interface_, canopen_object_dictionary, 
                                    node_id);

  LoggerSharedPtr logger = std::make_shared<Logger>(node);
  diagnostic_updater_.add(node_name,
                          std::bind(&Logger::log, logger, std::placeholders::_1));
  nodes_->add(node);
  nodes_lookup_.insert(std::make_pair(node_name, node));

  std::shared_ptr<canopen::EMCYHandler> emcy =
        std::make_shared<canopen::EMCYHandler>(interface_, node->getStorage());
  emcy_handlers_->add(emcy);
  logger->add(emcy);

  loggers_.push_back(logger);
  // CANopen IO profile (401)
  io_profile_subcomponents_.push_back(std::make_shared<IOSubcomponent>(
    this->get_node_base_interface(),
    this->get_node_logging_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface(),
    this->get_node_services_interface(),
    node_name,
    node
  ));

  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating canopen_chain");

  if (getLayerState() > Off) {
    RCLCPP_WARN(this->get_logger(), "Already initialized!");
  }

  canopen::LayerReport status;
  try {
    init(status);
  } catch (const std::exception &e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    status.error(info);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception while initializng CAN Layers");
    status.error("Unknown exception while initializng CAN Layers");
  }

  RCLCPP_INFO(this->get_logger(), "CAN init has finished!");

  //shutdown(status);

  // TODO(sam): Make optional implementation using a dedicated thread and "sleep_until". Don't use boost?
  update_periodic_timer_ = create_wall_timer(
    std::chrono::milliseconds(update_period_ms_), std::bind(&CanopenChainComponent::update_callback, this));

  std::flush(std::cout);
  if (getLayerState() == Off) {
    RCLCPP_ERROR(this->get_logger(), "Could not initialize main CAN Layer! Is the CAN network available? Is device power on?");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  for(auto const& io_profile_subcomponent: io_profile_subcomponents_)
  {
    io_profile_subcomponent->activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CanopenChainComponent::update_callback()
{
  canopen::LayerStatus layer_status;
  try {
    read(layer_status);
    write(layer_status);

    if (!layer_status.bounded<canopen::LayerStatus::Warn>()) {
      RCLCPP_ERROR_ONCE(this->get_logger(), layer_status.reason());
      // NOTE(sam): node starts printing sdo, no response messages
      // and becomes unresponsive to callbacks if one node is shut off
      // just deactivate on warnings for now...
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    } else if (!layer_status.bounded<canopen::LayerStatus::Ok>()) {
      RCLCPP_WARN_ONCE(this->get_logger(), layer_status.reason());
    }
  } catch (const canopen::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), boost::diagnostic_information(e));
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating canopen_chain");

  canopen::LayerReport status;
  try {
    this->Layer::shutdown(status);
  } catch (const std::exception &e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    status.error(info);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown exception while shutting down CAN Layers");
    status.error("Unknown exception while shutting down CAN Layers");
  }

  update_periodic_timer_->cancel();

  for(auto const& io_profile_subcomponent: io_profile_subcomponents_)
  {
    io_profile_subcomponent->deactivate();
  }

  // Attempt to recover all nodes
  for (auto const& canopen_node_entry: nodes_lookup_)
  {
    canopen::LayerStatus layer_status;
    canopen_node_entry.second->recover(layer_status);
  }

  std::flush(std::cout);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CanopenChainComponent::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up canopen_chain");

  // FIXME(sam): Not able to completly clean up after and recreate canopen nodes at this time.

  srv_list_object_dictionaries_.reset();

  // NOTE(sam): Should remove all "layers" added with this->add(), not sure it works...
  destroy();

  nodes_.reset();
  nodes_lookup_.empty();
  io_profile_subcomponents_.empty();

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
