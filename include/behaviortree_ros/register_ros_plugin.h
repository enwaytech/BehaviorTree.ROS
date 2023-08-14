// Copyright (c) 2023 Davide Faconti
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

#pragma once

#include <ros/node_handle.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <filesystem>

// Use this macro to generate a plugin for:
//
// - BT::RosActionNode
// - BT::RosServiceNode
// - BT::RosServiceNode
// - BT::RosTopicPubNode
// - BT::RosTopicSubNode
//
// - First argument: type to register (name of the class)
// - Second argument: string with the registration name
//
// Usage example:
//   CreateRosNodePlugin(MyClassName, "MyClassName");

#define CreateRosNodePlugin(TYPE, REGISTRATION_NAME)                                                                   \
  BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory, ros::NodeHandle& node_handle)       \
  {                                                                                                                    \
    BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config)             \
    {                                                                                                                  \
      return std::make_unique<TYPE>(node_handle, name, config);                                                        \
    };                                                                                                                 \
                                                                                                                       \
    BT::TreeNodeManifest manifest;                                                                                     \
    manifest.type = BT::getType<TYPE>();                                                                               \
    manifest.ports = TYPE::providedPorts();                                                                            \
    manifest.registration_ID = REGISTRATION_NAME;                                                                      \
                                                                                                                       \
    factory.registerBuilder(manifest, builder);                                                                        \
  }

/**
 * @brief RegisterRosNode function used to load a plugin and register
 * the containing Node definition.
 *
 * @param factory     the factory where the node should be registered.
 * @param filepath    path to the plugin.
 * @param node_handle node handleto pass to the instances of the Node.
 */
inline void
RegisterRosNode(BT::BehaviorTreeFactory& factory, const std::filesystem::path& filepath, ros::NodeHandle& node_handle)
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory&, ros::NodeHandle&);
  auto func = (Func)loader.getSymbol("BT_RegisterRosNodeFromPlugin");
  func(factory, node_handle);
}
