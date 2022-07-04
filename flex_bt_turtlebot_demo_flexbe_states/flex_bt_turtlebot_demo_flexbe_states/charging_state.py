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

from __future__ import division
import math

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState


class ChargingState(EventState):
    '''
    Simulated charging state
    -- charger_topic           string   Topic to publish charging with
    -- battery_topic           string   Topic to subscribe for battery status
    -- battery_threshold       double   Battery percentage to charge to

    <= done                         Given time has passed.
    '''

    def __init__(self, charger_topic, battery_topic, battery_threshold):
        super(ChargingState, self).__init__(outcomes = ['done', 'failed'])

        ProxyPublisher._initialize(ChargingState._node)
        ProxySubscriberCached._initialize(ChargingState._node)

        self._charger_topic = charger_topic
        self._battery_topic = battery_topic

        self._pub = ProxyPublisher({self._charger_topic: Bool})
        self._battery_sub = ProxySubscriberCached({self._battery_topic:  BatteryState})

        self._battery_threshold = battery_threshold
        self._battery_level = 0
        self._return = None


    def execute(self, userdata):
        if self._return:
            return self._return

        if self._battery_sub.has_msg(self._battery_topic):
            self._battery_level = self._battery_sub.get_last_msg(self._battery_topic).percentage

            if self._battery_level < self._battery_threshold:
                bool_msg = Bool(data=True)
                self._pub.publish(self._charger_topic, bool_msg)
            else:
                bool_msg = Bool(data=False)
                self._pub.publish(self._charger_topic, bool_msg)
                self._return = 'done'
                Logger.loginfo('%s  Done charging - battery level at %f' % (self.name, self._battery_level))
                return self._return
                


    def on_enter(self, userdata):
        self._return = None
        if self._battery_sub.has_msg(self._battery_topic):
            self._battery_level = self._battery_sub.get_last_msg(self._battery_topic).percentage
        else:
            self._battery_level = 0
