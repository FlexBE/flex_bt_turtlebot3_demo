#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022
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

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import PoseStamped

class GetWaypointsState(EventState):
    """
    Grabs the most recent published PoseStampeds to use as waypoints.

    Requires at least 2 waypoints to be defined, but continues to collect
    published goals until the specified timeout has elapsed since the last goal
    was recieved.

    -- timeout       double           Timeout
    -- topic         String           The topic to subscribe to

    #> waypoints     PoseStamped[]    A list of waypoints

    <= done          Waypoints PoseStamped[] is available.
    <= canceled      Cancel waypoint acceptance and return no user waypoints

    """

    def __init__(self, timeout, topic = 'move_base_simple/goal'):
        """Constructor"""
        super(GetWaypointsState, self).__init__(outcomes=['done', 'canceled'], output_keys=['waypoints'])

        ProxySubscriberCached._initialize(GetWaypointsState._node)

        self._topic      = topic
        self._sub        = ProxySubscriberCached({self._topic: PoseStamped})
        self._waypoints  = []

        self._timeout    = Duration(seconds=timeout)
        self._start_time = None
        self._return = None

    def execute(self, userdata):
        if self._return:
            # Handle case if blocked transition
            userdata.waypoints = self._waypoints
            self._return = 'done'

        # Process current messages in queue, and then exit execute so as
        # not to block execute loop while waiting on user input
        while self._sub.has_msg(self._topic):
            Logger.loginfo('%s  Received new waypoint ' % (self.name))
            self._waypoints.append(self._sub.get_last_msg(self._topic))
            self._sub.remove_last_msg(self._topic)
            if len(self._waypoints) < 2:
                Logger.loghint('%s  Input a new 2D Nav goal (e.g. via RViz) as waypoint (2 required!) ' % (self.name))

            # Refresh the start time everytime a new goal is set
            # Allows setting many goals
            self._start_time = self._node.get_clock().now()

        if len(self._waypoints) > 1:
           if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
                # Collected enough, and timeout
                Logger.loginfo('%s  Finalized waypoint list with %d points' % (self.name, len(self._waypoints)))
                userdata.waypoints = self._waypoints
                self._return = 'done'
                return self._return
        else:
           if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
                Logger.loghint('%s  Input a new 2D Nav goal (e.g. via RViz) as waypoint (2 required!) ' % (self.name))
                self._start_time = self._node.get_clock().now()

    def on_enter(self, userdata):
        userdata.waypoints = None
        self._return = None
        self._waypoints = []
        self._start_time = self._node.get_clock().now()
        if self._sub.has_msg(self._topic):
            Logger.loginfo('%s  Clearing prior goal information - start fresh' % (self.name))
            self._sub.remove_last_msg(self._topic)
        Logger.loghint('%s  Input a new 2D Nav goal (e.g. via RViz) ' % (self.name))

    def on_exit(self, userdata):
        if self._return is None:
            if self._manual_transition_requested == "done":
                userdata.waypoints = self._waypoints
                self._return = 'done'
                Logger.loginfo('%s  Accepting %d waypoints with forced transition' % (self.name, len(self._waypoints)))
            else:
                Logger.loginfo('%s  Ignoring %d waypoints with forced transition %s' % (self.name, len(self._waypoints), self._manual_transition_requested))
