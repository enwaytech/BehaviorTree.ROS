// Copyright (c) 2022 SysDesign S.r.l.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_PARAM_NODE_HPP_
#define BEHAVIOR_TREE_BT_PARAM_NODE_HPP_

#include <ros/ros.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>


namespace BT {


template<class ParamT>
class RosParamNode : public BT::SyncActionNode {
protected:

  ros::NodeHandle node;

  RosParamNode(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
  : BT::SyncActionNode(name, conf), node(nh) { }

public:

  using BaseClass = RosParamNode<ParamT>;
  using ParamType = ParamT;

  RosParamNode() = delete;
  virtual ~RosParamNode() = default;

  static PortsList
  providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
        InputPort<std::string>("param_name", "name of the parameter on the Parameter Server"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static PortsList
  providedPorts()
  {
    return providedBasicPorts({});
  }
};


template <class DerivedT> static
void RegisterRosParam(BT::BehaviorTreeFactory &factory, const std::string &registration_ID, ros::NodeHandle &node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto &basic_ports = RosParamNode<typename DerivedT::ParamType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());

  factory.registerBuilder(manifest, builder);
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_TOPIC_NODE_HPP_
