#ifndef CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_HELPERS

#include <canopen_master/canopen.hpp>
#include <canopen_master/can_layer.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>

namespace canopen_chain_node
{

class Logger : public canopen::DiagGroup<canopen::Layer>
{
  const canopen::NodeSharedPtr node_;

  std::vector<std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>> entries_;

  static void log_entry(
    diagnostic_updater::DiagnosticStatusWrapper & stat, uint8_t level,
    const std::string & name, std::function<std::string()> getter)
  {
    if (stat.level >= level) {
      try {
        stat.add(name, getter());
      } catch (...) {
        stat.add(name, "<ERROR>");
      }
    }
  }

public:
  explicit Logger(canopen::NodeSharedPtr node)
  : node_(node) {add(node_);}

  bool add(uint8_t level, const std::string & key, bool forced)
  {
    try {
      canopen::ObjectDict::Key k(key);
      const canopen::ObjectDict::EntryConstSharedPtr entry = node_->getStorage()->dict_->get(k);
      std::string name = entry->desc.empty() ? key : entry->desc;
      entries_.push_back(std::bind(log_entry, std::placeholders::_1, level, name,
        node_->getStorage()->getStringReader(k, !forced)));
      return true;
    } catch (std::exception & e) {
      // RCLCPP_ERROR(this->get_logger(), boost::diagnostic_information(e));
      return false;
    }
  }

  template<typename T>
  void add(const std::shared_ptr<T> & n)
  {
    DiagGroup::add(std::static_pointer_cast<canopen::Layer>(n));
  }

  virtual void log(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (node_->getState() == canopen::Node::Unknown) {
      stat.summary(stat.WARN, "Not initailized");
    } else {
      canopen::LayerReport r;
      diag(r);
      if (r.bounded<canopen::LayerStatus::Unbounded>()) {  // valid
        stat.summary(r.get(), r.reason());
        for (std::vector<std::pair<std::string, std::string>>::const_iterator it =
          r.values().begin(); it != r.values().end(); ++it)
        {
          stat.add(it->first, it->second);
        }
        for (size_t i = 0; i < entries_.size(); ++i) {
          entries_[i](stat);
        }
      }
    }
  }

  virtual ~Logger() {}
};
typedef std::shared_ptr<Logger> LoggerSharedPtr;

    
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

}   // namespace canopen_chain_node

#endif // CANOPEN_CHAIN_NODE__CANOPEN_CHAIN_HELPERS