#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flex_bt_flexbe_states.bt_execute_goal_state import BtExecuteGoalState
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState
from flex_bt_flexbe_states.bt_loader_state import BtLoaderState
from flex_bt_turtlebot_demo_flexbe_states.charging_state import ChargingState
from flex_bt_turtlebot_demo_flexbe_states.check_battery_life_state import CheckBatteryLifeState
from flex_bt_turtlebot_demo_flexbe_states.get_waypoints_state import GetWaypointsState
from flex_bt_turtlebot_demo_flexbe_states.send_waypoints_state import SendWaypointsState
from flex_nav_flexbe_states.get_pose_state import GetPoseState
from flex_nav_flexbe_states.log_path_state import LogPathState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Feb 04 2022
@author: Josh Zutell
'''
class TurtlebotPatrolSM(Behavior):
	'''
	Using Flexible Navigation with Behavior Trees to control the Turtlebot
	'''


	def __init__(self, node):
		super(TurtlebotPatrolSM, self).__init__()
		self.name = 'Turtlebot Patrol'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		BtLoaderState.initialize_ros(node)
		BtExecuteGoalState.initialize_ros(node)
		BtExecuteState.initialize_ros(node)
		ChargingState.initialize_ros(node)
		CheckBatteryLifeState.initialize_ros(node)
		GetPoseState.initialize_ros(node)
		GetWaypointsState.initialize_ros(node)
		LogPathState.initialize_ros(node)
		LogState.initialize_ros(node)
		OperatorDecisionState.initialize_ros(node)
		SendWaypointsState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:672 y:77, x:377 y:485
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.min_battery = 0.20

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:623 y:85, x:296 y:227
		_sm_setupbehavior_0 = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['charging_location'])

		with _sm_setupbehavior_0:
			# x:96 y:67
			OperatableStateMachine.add('LoadBTs',
										BtLoaderState(bt_topic="bt_loader", filepaths=["flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_plan_path.xml", "flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_navigate_w_battery_check.xml", "flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_recovery_fallback.xml", "flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml"], timeout=5.0),
										transitions={'done': 'GetChargingLocation', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:388 y:68
			OperatableStateMachine.add('GetChargingLocation',
										GetPoseState(topic='move_base_simple/goal'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'goal': 'charging_location'})


		# x:241 y:262, x:934 y:33, x:506 y:257
		_sm_navigate_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'low_battery'], input_keys=['min_battery'])

		with _sm_navigate_1:
			# x:159 y:128
			OperatableStateMachine.add('FollowPath',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_navigate_w_battery_check.xml", goal_id="min_battery", goal_msg_type="double", request_id="battery_percentage", request_msg_pkg="", request_msg_type="double"),
										transitions={'done': 'finished', 'canceled': 'finished', 'failed': 'IsBatteryLow'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'min_battery', 'data': 'battery_life'})

			# x:364 y:128
			OperatableStateMachine.add('IsBatteryLow',
										CheckBatteryLifeState(battery_threshold=0.2),
										transitions={'true': 'low_battery', 'false': 'failed'},
										autonomy={'true': Autonomy.Low, 'false': Autonomy.Low},
										remapping={'battery_life': 'battery_life'})


		# x:789 y:162, x:243 y:37, x:1284 y:192, x:1278 y:328
		_sm_patrol_2 = OperatableStateMachine(outcomes=['failed', 'canceled', 'low_battery', 'finished'], input_keys=['waypoints', 'min_battery'], output_keys=['waypoints'])

		with _sm_patrol_2:
			# x:203 y:251
			OperatableStateMachine.add('ContinuePatrolling',
										OperatorDecisionState(outcomes=["yes", "no"], hint=None, suggestion="yes"),
										transitions={'yes': 'SendWaypoint', 'no': 'canceled'},
										autonomy={'yes': Autonomy.Low, 'no': Autonomy.Full})

			# x:1026 y:28
			OperatableStateMachine.add('LogPath',
										LogPathState(),
										transitions={'done': 'Navigate'},
										autonomy={'done': Autonomy.Low},
										remapping={'plan': 'plan'})

			# x:1041 y:250
			OperatableStateMachine.add('Navigate',
										_sm_navigate_1,
										transitions={'finished': 'ContinuePatrolling', 'failed': 'failed', 'low_battery': 'low_battery'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'low_battery': Autonomy.Inherit},
										remapping={'min_battery': 'min_battery'})

			# x:740 y:28
			OperatableStateMachine.add('Plan',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_plan_path.xml", goal_id="goal", goal_msg_type="PoseStamped", request_id="path", request_msg_pkg="nav_msgs", request_msg_type="Path"),
										transitions={'done': 'LogPath', 'canceled': 'ContinuePatrolling', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'goal', 'data': 'plan'})

			# x:385 y:27
			OperatableStateMachine.add('SendWaypoint',
										SendWaypointsState(),
										transitions={'done': 'Plan', 'canceled': 'canceled'},
										autonomy={'done': Autonomy.Low, 'canceled': Autonomy.Low},
										remapping={'waypoints': 'waypoints', 'goal': 'goal'})


		# x:775 y:84, x:397 y:224
		_sm_charge_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['charging_location'])

		with _sm_charge_3:
			# x:136 y:79
			OperatableStateMachine.add('NavigateToCharger',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml", goal_id="goal", goal_msg_type="PoseStamped", request_id="", request_msg_pkg="", request_msg_type=""),
										transitions={'done': 'Charge', 'canceled': 'Charge', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'charging_location', 'data': 'data'})

			# x:492 y:79
			OperatableStateMachine.add('Charge',
										ChargingState(charger_topic="robot_charger", battery_topic="battery_status", battery_threshold=0.95),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off})



		with _state_machine:
			# x:86 y:196
			OperatableStateMachine.add('SetupBehavior',
										_sm_setupbehavior_0,
										transitions={'finished': 'GetWaypoints', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'charging_location': 'charging_location'})

			# x:999 y:214
			OperatableStateMachine.add('Charge',
										_sm_charge_3,
										transitions={'finished': 'Patrol', 'failed': 'Recovery'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'charging_location': 'charging_location'})

			# x:346 y:296
			OperatableStateMachine.add('GetWaypoints',
										GetWaypointsState(timeout=5.0, topic='move_base_simple/goal'),
										transitions={'done': 'Patrol', 'canceled': 'CancelBehavior'},
										autonomy={'done': Autonomy.Low, 'canceled': Autonomy.Off},
										remapping={'waypoints': 'waypoints'})

			# x:645 y:286
			OperatableStateMachine.add('Patrol',
										_sm_patrol_2,
										transitions={'failed': 'Recovery', 'canceled': 'GetWaypoints', 'low_battery': 'Charge', 'finished': 'Patrol'},
										autonomy={'failed': Autonomy.Inherit, 'canceled': Autonomy.Inherit, 'low_battery': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'waypoints': 'waypoints', 'min_battery': 'min_battery'})

			# x:959 y:462
			OperatableStateMachine.add('Recovery',
										BtExecuteState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot_demo_bringup/behavior_trees/nav2_recovery_fallback.xml"),
										transitions={'done': 'Patrol', 'canceled': 'Patrol', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off})

			# x:247 y:44
			OperatableStateMachine.add('Restart',
										OperatorDecisionState(outcomes=["waypoints", "charging", "exit"], hint="Reenter only waypoints, or both, or exit", suggestion="waypoints"),
										transitions={'waypoints': 'GetWaypoints', 'charging': 'SetupBehavior', 'exit': 'finished'},
										autonomy={'waypoints': Autonomy.High, 'charging': Autonomy.High, 'exit': Autonomy.Off})

			# x:470 y:149
			OperatableStateMachine.add('CancelBehavior',
										LogState(text="Canceled GetWaypoints - Try again", severity=Logger.REPORT_HINT),
										transitions={'done': 'Restart'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
