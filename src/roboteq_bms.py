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

from rcomponent import RComponent, DEFAULT_FREQ # esto podria estar como limites

import robotnik_msgs.msg

import std_msgs.msg

import serial

from serial import SerialException

class roboteq_bmsComponent(RComponent):
	def __init__(self):
		RComponent.__init__(self)

		self.port = '/dev/ttyUSB_ROBOTEQ_BMS'
		self.read_errors = 0
		self.bat_level= 0.0
		self.voltage = 0.0
		self.current = 0.0
		self.cell_voltages = ''
		self.cell_currents = ''
		self.status_flags = ''
		self.battery_status_message = robotnik_msgs.msg.BatteryStatus()
		self.bms_temperature = std_msgs.msg.Int32()
		self.cell_voltages_msg = std_msgs.msg.String()
		self.cell_currents_msg = std_msgs.msg.String()
		self.status_flags_msg = std_msgs.msg.String()
		self.serial_device = None

	def setup(self):

		RComponent.setup(self)

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


	def shutdown(self):
		RComponent.shutdown(self)

		rospy.loginfo("roboteq_bms::shutdown")

		if self.serial_device != None and not self.serial_device.closed:
			self.serial_device.close()

		return 0

	def rosSetup(self):
		RComponent.rosSetup(self)

		self.bat_data_publisher_ = rospy.Publisher('~battery_status', robotnik_msgs.msg.BatteryStatus, queue_size=100)
		self.bms_temp_publisher_ = rospy.Publisher('~bms_temperature', std_msgs.msg.Int32, queue_size=100)
		self.status_flags_publisher_ = rospy.Publisher('~status_flags', std_msgs.msg.String, queue_size=100)
		self.cell_voltages_publisher_ = rospy.Publisher('~cell_voltages', std_msgs.msg.String, queue_size=100)
		self.cell_currents_publisher_ = rospy.Publisher('~cell_currents', std_msgs.msg.String, queue_size=100)

	def rosShutdown(self):
		if self.running:
			rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
			return -1
		elif not self.ros_initialized:
			rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
			return -1

		RComponent.rosShutdown(self)

		#self.set_digital_outputs_service_.shutdown()
		#self.get_sw_version_service_.shutdown()
		self.bat_data_publisher_.unregister()
		self.temperature_bms_publisher_.unregister()
		self.status_flags_publisher_unregister()
		self.cell_voltages_publisher_unregister()
		self.cell_currents_publisher_unregister()

	def readyState(self):
		if not rospy.is_shutdown():

			emptys = []
			self.writeToSerialDevice("?BSC" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.bat_level = float(line_read.partition("BSC=")[2])
				self.battery_status_message.level = self.bat_level
				emptys.append(False)
			else:
				emptys.append(True)

			
			self.writeToSerialDevice("?A 1" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.current = float(line_read.partition("A=")[2])
				self.battery_status_message.current = self.current*0.01
				emptys.append(False)
			else:
				emptys.append(True)
			
			self.writeToSerialDevice("?V 1" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.voltage = float(line_read.partition("V=")[2])
				self.battery_status_message.voltage = self.voltage*0.01
				emptys.append(False)
			else:
				emptys.append(True)

			#Extras Miguel
			self.writeToSerialDevice("?T 1" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				try:
					temperature = line_read.partition("T=")[2]
					self.temperature = float(temperature)
					self.bms_temperature.data = self.temperature
					emptys.append(False)
				except ValueError:
					rospy.logerror("Could not convert string to float: %s" % (temperature))
			else:
				emptys.append(True)
			
			self.writeToSerialDevice("?BMF" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.status_flags = line_read.partition("BMF=")[2]
				self.status_flags_msg.data = self.status_flags
				emptys.append(False)
			else: 
				emptys.append(True)
			
			self.writeToSerialDevice("?V" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.cell_voltages = line_read.partition("V=")[2]
				self.cell_voltages_msg.data = self.cell_voltages
				emptys.append(False)
			else:
				emptys.append(True)
			
			self.writeToSerialDevice("?A" + "\r")
			line_read = str(self.readFromSerialDevice())
			if line_read != '':
				self.cell_currents = line_read.partition("A=")[2]
				self.cell_currents_msg.data = self.cell_currents
				emptys.append(False)
			else:
				emptys.append(True)
			
			if all(emptys):
				print('all are empty, something wrong with bms')
			elif any(emptys):
				print('some emtpy, im missing messages')



	def rosPublish(self):
		RComponent.rosPublish(self)
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

def main():
    rospy.init_node("roboteq_bms")

    _name = rospy.get_name().replace('/','')

    roboteq_bms_node = roboteq_bmsComponent()

    roboteq_bms_node.start()

if __name__ == "__main__":
	main()
