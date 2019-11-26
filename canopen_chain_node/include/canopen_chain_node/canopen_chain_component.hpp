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

#include <canopen_master/canopen.hpp>
#include <canopen_master/can_layer.hpp>
#include <canopen_msgs/srv/list_object_dictionaries.hpp>
#include <canopen_msgs/srv/get_object.hpp>
#include <canopen_msgs/msg/object_dictionary.hpp>
#include <canopen_msgs/msg/object_description.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <socketcan_interface/interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <string>
// #include <utility>
// #include <map>
// #include <sys/stat.h>

namespace canopen_chain_node
{

class GuardedClassLoaderList
{
public:
  typedef std::shared_ptr<pluginlib::ClassLoaderBase> ClassLoaderBaseSharedPtr;
  static void addLoader(ClassLoaderBaseSharedPtr b)
  {
    guarded_loaders().push_back(b);
  }
  ~GuardedClassLoaderList()
  {
    guarded_loaders().clear();
  }

private:
  static std::vector<ClassLoaderBaseSharedPtr> & guarded_loaders()
  {
    static std::vector<ClassLoaderBaseSharedPtr> loaders;
    return loaders;
  }
};

template<typename T>
class GuardedClassLoader
{
  typedef pluginlib::ClassLoader<T> Loader;
  std::shared_ptr<Loader> loader_;

public:
  typedef std::shared_ptr<T> ClassSharedPtr;
  GuardedClassLoader(const std::string & package, const std::string & allocator_base_class)
  : loader_(new Loader(package, allocator_base_class))
  {
    GuardedClassLoaderList::addLoader(loader_);
  }
  ClassSharedPtr createInstance(const std::string & lookup_name)
  {
    return loader_->createUniqueInstance(lookup_name);
  }
};

template<typename T>
class ClassAllocator : public GuardedClassLoader<typename T::Allocator>
{
public:
  typedef std::shared_ptr<T> ClassSharedPtr;
  ClassAllocator(const std::string & package, const std::string & allocator_base_class)
  : GuardedClassLoader<typename T::Allocator>(package, allocator_base_class) {}
  template<typename T1>
  ClassSharedPtr allocateInstance(const std::string & lookup_name, const T1 & t1)
  {
    return this->createInstance(lookup_name)->allocate(t1);
  }
  template<typename T1, typename T2>
  ClassSharedPtr allocateInstance(const std::string & lookup_name, const T1 & t1, const T2 & t2)
  {
    return this->createInstance(lookup_name)->allocate(t1, t2);
  }
  template<typename T1, typename T2, typename T3>
  ClassSharedPtr allocateInstance(
    const std::string & lookup_name, const T1 & t1, const T2 & t2,
    const T3 & t3)
  {
    return this->createInstance(lookup_name)->allocate(t1, t2, t3);
  }
};


class CanopenChainComponent : GuardedClassLoaderList, public canopen::LayerStack, public rclcpp_lifecycle::LifecycleNode
{
  GuardedClassLoader<can::DriverInterface> driver_loader_;
  ClassAllocator<canopen::Master> master_allocator_;

public:
  CanopenChainComponent();

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

  int update_period_ms_;

  std::string default_eds_pkg_;
  std::string default_eds_file_;

  canopen::MasterSharedPtr master_;
  canopen::SyncLayerSharedPtr sync_;
  can::DriverInterfaceSharedPtr interface_;

  std::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node>> nodes_;
  std::map<std::string, canopen::NodeSharedPtr> nodes_lookup_;

  rclcpp::TimerBase::SharedPtr update_periodic_timer_;

  rclcpp::Service<canopen_msgs::srv::ListObjectDictionaries>::SharedPtr srv_list_object_dictionaries_;
  rclcpp::Service<canopen_msgs::srv::GetObject>::SharedPtr srv_get_object_;

  void update_callback();

  bool configure_bus();
  bool configure_sync();
  bool configure_heartbeat();
  bool configure_defaults();
  bool configure_nodes();
  bool configure_node(std::string node_name);

  void handle_list_objet_dictionaries(
    const std::shared_ptr<canopen_msgs::srv::ListObjectDictionaries::Request> request,
    std::shared_ptr<canopen_msgs::srv::ListObjectDictionaries::Response> response);

  void handle_get_object(
      const std::shared_ptr<canopen_msgs::srv::GetObject::Request> request,
      std::shared_ptr<canopen_msgs::srv::GetObject::Response> response);
  };
 
}  // namespace canopen_chain_node

#endif  // CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_COMPONENT_HPP_
