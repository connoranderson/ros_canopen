# Copyright 2019 Dyno Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Laucher for canopen_chain with lifecycle autostart."""

import os

from ament_index_python import get_package_share_directory

import launch
from launch.actions import EmitEvent

import launch_ros
from launch_ros.actions import LifecycleNode

import lifecycle_msgs.msg


def generate_launch_description():
    """Generate launch description."""
    test_params = os.path.join(
        get_package_share_directory('canopen_chain_node'),
        'test/config/chain_node_params.yaml'
    )

    ld = launch.LaunchDescription()

    chain_node = LifecycleNode(
        node_name='canopen_chain',
        package='canopen_chain_node',
        node_executable='manual_composition',
        output='screen',
        parameters=[
            test_params
        ]
    )

    # Make the node take the 'configure' transition.
    emit_event_request_that_chain_node_does_configure_transition = EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(chain_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # When the node reaches the 'inactive' state, make it 'activate'
    register_event_handler_for_chain_node_reaches_inactive_state = launch.actions.RegisterEventHandler(  # noqa
      launch_ros.event_handlers.OnStateTransition(
        target_lifecycle_node=chain_node, goal_state='inactive',
        entities=[
         EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
           lifecycle_node_matcher=launch.events.matches_action(chain_node),
           transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )),
        ],
      )
    )

    # Register nodes and event handlers before starting 'configure' transition
    ld.add_action(register_event_handler_for_chain_node_reaches_inactive_state)
    ld.add_action(chain_node)
    ld.add_action(emit_event_request_that_chain_node_does_configure_transition)

    return ld
