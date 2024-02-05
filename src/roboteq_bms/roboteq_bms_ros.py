#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import rospy
import time, threading
from rcomponent.rcomponent import *
from robotnik_msgs.msg import BatteryStatus, StringStamped
from std_msgs.msg import Int32, String
import serial
from serial import SerialException


class RoboteqBMS(RComponent):
    """
    
    """

    def __init__(self):
        self.port = '/dev/ttyUSB_BMS'
        self.read_errors = 0
        self.bat_level= 0.0
        self.voltage = 0.0
        self.current = 0.0
        self.cell_voltages = ''
        self.cell_currents = ''
        self.status_flags = ''
        self.battery_status_message = BatteryStatus()
        self.bms_temperature = Int32()
        self.cell_voltages_msg = String()
        self.cell_currents_msg = String()
        self.status_flags_msg = String()
        self.serial_device = None
        self.start_time = 0
        self.charge_started = False
        self.status = String()
        super().__init__()
    def ros_read_params(self):
        """Gets params from param server"""
        super().ros_read_params()
        self.port = rospy.get_param('~port', '/dev/ttyUSB_BMS')

    def ros_setup(self):
        """Creates and inits ROS components"""

        super().ros_setup()

                # Publisher
        self.status_pub = rospy.Publisher(
            '~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher(
            '~status_stamped', StringStamped, queue_size=10)

        self.bat_data_publisher_ = rospy.Publisher('~data', BatteryStatus, queue_size=100)
        self.bms_temp_publisher_ = rospy.Publisher('~temperature', Int32, queue_size=100)
        self.status_flags_publisher_ = rospy.Publisher('~status_flags', String, queue_size=100)
        self.cell_voltages_publisher_ = rospy.Publisher('~cell_voltages', String, queue_size=100)
        self.cell_currents_publisher_ = rospy.Publisher('~cell_currents', String, queue_size=100)

    def setup(self):
        super().setup()
        self.serial_device = serial.Serial(
			port= self.port,
			baudrate=115200,
			parity=serial.PARITY_NONE,
			stopbits=1,
			bytesize=8,
			timeout=0.1,
			xonxoff=False,
			dsrdtr=False,
		    rtscts=False
		    )

    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        if not rospy.is_shutdown():
            self.read()
            
        return super().ready_state()

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        rospy.loginfo("roboteq_bms::shutdown")

        if self.serial_device != None and not self.serial_device.closed:
            self.serial_device.close()
            
        return super().shutdown()
    
    def ros_shutdown(self):
        if self._running:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self._ros_initialized:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        super().ros_shutdown()

        #self.set_digital_outputs_service_.shutdown()
        #self.get_sw_version_service_.shutdown()
        self.bat_data_publisher_.unregister()
        self.bms_temp_publisher_.unregister()
        self.status_flags_publisher_.unregister()
        self.cell_voltages_publisher_.unregister()
        self.cell_currents_publisher_.unregister()

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return super().switch_to_state(new_state)
    
    def ros_publish(self):
        super().ros_publish()
        self.bat_data_publisher_.publish(self.battery_status_message)
        self.bms_temp_publisher_.publish(self.bms_temperature)
        self.status_flags_publisher_.publish(self.status_flags_msg)
        self.cell_voltages_publisher_.publish(self.cell_voltages_msg)
        self.cell_currents_publisher_.publish(self.cell_currents_msg)

    def writeToSerialDevice(self, data):
        bytes_written = self.serial_device.write(data)
        return bytes_written

    def readFromSerialDevice(self):
        try:
            data_read = self.serial_device.readline()
        except SerialException as e:
            rospy.logwarn(e)
            return

        return data_read

    def read(self):
        emptys = []
        string = "?BSC" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())
        try:
            if line_read != '':
                self.bat_level = float(line_read.partition("BSC=")[2].split('\r')[0])
                self.battery_status_message.level = self.bat_level
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?BSC - response (%s): %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?A 1" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                self.current = -float(line_read.partition("A=")[2].split('\r')[0])
                self.battery_status_message.current = self.current*0.01
                if self.battery_status_message.current < -0.5:
                    self.battery_status_message.is_charging = True
                    if not self.charge_started:
                        self.charge_started = True
                        self.start_time = time.time()
                        self.battery_status_message.time_charging = 0
                    else:
                        actual = time.time()
                        self.battery_status_message.time_charging = int(round((actual - self.start_time)/60, 1)) 
                else:
                    if self.charge_started:
                        self.charge_started = False
                        self.battery_status_message.time_charging = 0
                    self.battery_status_message.is_charging = False

                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?A 1 - response (%s):: %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?V 1" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                self.voltage = float(line_read.partition("V=")[2].split('\r')[0])
                self.battery_status_message.voltage = self.voltage*0.01
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?V 1 - response (%s):: %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?T 1" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                temperature = line_read.partition("T=")[2].split('\r')[0]
                self.temperature = float(temperature)
                self.bms_temperature.data = self.temperature
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?T 1 - response (%s):: %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?BMF" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                self.status_flags = line_read.partition("BMF=")[2].split('\r')[0]
                self.status_flags_msg.data = self.status_flags
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?BMF - response(%s): %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?V" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                self.cell_voltages = line_read.partition("V=")[2].split('\r')[0]
                self.cell_voltages_msg.data = self.cell_voltages
                cell_voltages_list = self.cell_voltages.split(":")
                aux_list = []
                for i in range(len(cell_voltages_list)-3):
                    v = cell_voltages_list[i+3]
                    v = float(v)
                    aux_list.append(float(v/1000.0))
                self.battery_status_message.cell_voltages = aux_list
                
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?V - response(%s): %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        string = "?A" + "\r"
        self.writeToSerialDevice(string.encode())
        line_read = str(self.readFromSerialDevice().decode())

        try:
            if line_read != '':
                self.cell_currents = line_read.partition("A=")[2].split('\r')[0]
                self.cell_currents_msg.data = self.cell_currents
                emptys.append(False)
            else:
                emptys.append(True)
        except ValueError as e:
            rospy.logerr('%s::readyState: error reading ?A - response (%s): %s', rospy.get_name(), line_read, e)
            emptys.append(True)

        if all(emptys):
            rospy.logerr('%s::readyState: no response from bms', self._node_name)
            self.switch_to_state(State.FAILURE_STATE)
        elif any(emptys):
            rospy.logwarn('%s::readyState: some response msgs from bms are empty', self._node_name)