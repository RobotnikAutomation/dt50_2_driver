#!/usr/bin/env python3

from itertools import accumulate
import serial
from serial import SerialException
from rcomponent.rcomponent import RComponent

import rospy

from std_srvs.srv import Trigger, TriggerResponse
from rcomponent.rcomponent import *

class DT502Driver(RComponent):

    def __init__(self):
        self.port = '/dev/ttyUSB0' #'/dev/ttyUSB_DT50 
        self.serial_device = None

        self.communication_initialized = False

        self.read_distance_service = None
        self.reads_per_request = 1

        RComponent.__init__(self)
    
    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self.port = rospy.get_param('~port', self.port)
        self.reads_per_request = rospy.get_param('~reads_per_request', self.reads_per_request)

    def setup(self):
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

        RComponent.setup(self)

    def ros_setup(self):

        RComponent.ros_setup(self)

        self.read_distance_service = rospy.Service('~read_distance', Trigger, self.read_distance_service_cb)

        #self.command_status_pub = rospy.Publisher(
        #    '~status', CommandManagerStatus, queue_size=1)
        
        return 0
    
    def ready_state(self):

        if self.communication_initialized == False:
            self.communication_initialized = self.initialize_communication()
            self.serial_device.close()
            rospy.sleep(1.0)
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


        return
    
    def shutdown(self):
        # Turn off laser
        rospy.loginfo("Turns off laser")
        command= bytes('\x0A\x00\x02\x03\x00\x61\x00\x00\x01\x00', 'utf-8')
        self.write(command)

        rospy.loginfo("Sends shutdown communication message")
        command = bytes('\x24\x00\x02\x0F\x00\x00\x00\x11\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x20', 'utf-8')
        self.write(command)

        self.serial_device.close()
        
        RComponent.shutdown(self)
    
    def write(self, data):
        bytes_written = self.serial_device.write(data)
        rospy.loginfo("bytes_written: %i", bytes_written)
        return bytes_written
    
    def read(self):
        try:
            data_read = self.serial_device.readline()
        except SerialException as e:
            rospy.logwarn(e)
            return

        return data_read

    def initialize_communication(self):
        command = bytes('\x24\x00\x02\x0F\x00\x00\x01\x11\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x20', 'utf-8')
        self.write(command)
        rospy.loginfo("First communication message sent")
        #print(self.read())
        rospy.sleep(2.0)


        rospy.loginfo("Sends second communication message")
        command = bytes('\x07\x00\x02\x01\x10\x00\x08', 'utf-8')
        self.write(command)
        rospy.sleep(2.0)

        # Turn on laser
        rospy.loginfo("Turns on laser")
        command= bytes('\x0A\x00\x02\x03\x00\x61\x00\x00\x01\x01', 'utf-8')
        self.write(command)
        rospy.sleep(2.0)



        return True
    
    def read_distance_service_cb(self, msg):
        self.serial_device.flush()
        # Reads datas
        accumulated_distance = 0
        rospy.loginfo("Reads distance from laser")
        for x in range(0,self.reads_per_request):
            command = bytes('\x08\x00\x02\x02\x00\x6D\x00\x00', 'utf-8')
            distance = self.read()
            print(distance)
            current_distance = int.from_bytes(distance[6:8], 'big', signed= False)
            accumulated_distance += current_distance
            rospy.loginfo("Distance read (in mm): %i", current_distance)
            rospy.sleep(2.0)

        average_distance = accumulated_distance / (1.0 * self.reads_per_request)
        print(average_distance)
        
        

        return TriggerResponse()