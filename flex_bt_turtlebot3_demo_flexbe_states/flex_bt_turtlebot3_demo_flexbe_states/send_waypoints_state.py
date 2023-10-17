#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2016-2017
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import PoseStamped

class SendWaypointsState(EventState):
    """
    Sends the next waypoint to navigate to

    <# waypoints     PoseStamped[]    A list of waypoints

    #> waypoints     PoseStamped[]    A list of waypoints
    #> goal          PoseStamped      A goal from the list of waypoints

    <= done          Goal PoseStamped is available.
    <= canceled      Canceled sending a goal

    """

    def __init__(self):
        """Constructor"""
        super(SendWaypointsState, self).__init__(outcomes=['done', 'canceled'], input_keys=['waypoints'], output_keys=['waypoints', 'goal'])

        ProxySubscriberCached.initialize(SendWaypointsState._node)
        self._return = None

    def execute(self, userdata):
        return self._return

    def on_enter(self, userdata):
        if len(userdata.waypoints) > 0:
            userdata.goal = userdata.waypoints[0]

            # Move current goal from the start of the list to the end
            userdata.waypoints.pop(0)
            userdata.waypoints.append(userdata.goal)
            Logger.loginfo('%s  Passing next waypoint ' % (self.name))

            self._return  = 'done'
        else:
            self._return = 'canceled'
