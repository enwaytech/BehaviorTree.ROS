/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#ifndef BEHAVIOR_TREE_BT_SERVICE_SERVER_NODE_HPP_
#define BEHAVIOR_TREE_BT_SERVICE_SERVER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

template<class ServiceT>
class RosServiceServerNode : public BT::SyncActionNode
{
protected:
  ros::NodeHandle node_;
  ros::ServiceServer service_server_;

  RosServiceServerNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf)
    , node_(nh)
  {
    if (service_server_ == nullptr)
    {
      std::string service_name = getInput<std::string>("service_name").value();
      service_server_ = node_.advertiseService(service_name, &RosServiceServerNode::serviceCallback, this);
    }
  }

  BT::NodeStatus node_status = BT::NodeStatus::FAILURE;
  BT::NodeStatus
  tick() override
  {
    return node_status;
  }

public:
  using BaseClass = RosServiceServerNode<ServiceT>;
  using ServiceType = ServiceT;
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  RosServiceServerNode() = delete;
  virtual ~RosServiceServerNode() = default;

  static PortsList
  providedBasicPorts(PortsList addition)
  {
    PortsList basic = {InputPort<std::string>("service_name", "name of the ROS service"),
                       InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static PortsList
  providedPorts()
  {
    return providedBasicPorts({});
  }

  virtual bool serviceCallback(RequestType& request, ResponseType& response) = 0;
};

template<class DerivedT>
static void
RegisterRosServiceServer(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config)
  {
    return std::make_unique<DerivedT>(node_handle, name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosServiceServerNode<typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());

  factory.registerBuilder(manifest, builder);
}

} // namespace BT

#endif // BEHAVIOR_TREE_BT_SERVICE_SERVER_NODE_HPP_
