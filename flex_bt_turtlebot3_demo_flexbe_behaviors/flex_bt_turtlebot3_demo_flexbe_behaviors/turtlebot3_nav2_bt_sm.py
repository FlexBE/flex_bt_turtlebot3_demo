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
from flex_nav_flexbe_states.get_pose_state import GetPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Feb 04 2022
@author: Josh Zutell
'''
class Turtlebot3Nav2BTSM(Behavior):
	'''
	Using Flexible Navigation with a Behavior Tree to control the Turtlebot
	'''


	def __init__(self, node):
		super(Turtlebot3Nav2BTSM, self).__init__()
		self.name = 'Turtlebot3 Nav2 BT'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		BtExecuteGoalState.initialize_ros(node)
		GetPoseState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:706 y:141
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:174 y:131
			OperatableStateMachine.add('GetGoal',
										GetPoseState(topic='move_base_simple/goal'),
										transitions={'done': 'Navigate'},
										autonomy={'done': Autonomy.High},
										remapping={'goal': 'goal'})

			# x:447 y:132
			OperatableStateMachine.add('Navigate',
										BtExecuteGoalState(bt_topic="bt_executor", bt_file="flex_bt_turtlebot3_demo_bringup/behavior_trees/nav2_navigate_to_pose.xml", goal_id="goal", goal_msg_type="PoseStamped", request_id="", request_msg_pkg="", request_msg_type=""),
										transitions={'done': 'GetGoal', 'canceled': 'GetGoal', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'goal': 'goal', 'data': 'data'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
