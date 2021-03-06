#!/usr/bin/env python3
from urllib import response
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

import serial
from serial import SerialException
import statistics
from rcomponent.rcomponent import RComponent

from dt50_2_driver.srv import GetDistance, GetDistanceResponse

class DT502Driver(RComponent):

    def __init__(self):
        self.port = '/dev/ttyUSB0' #'/dev/ttyUSB_DT50 
        self.serial_device = None

        self.communication_initialized = False

        self.read_distance_service = None
        self.reads_per_request = 1

        self.startFirstCommunication_cmd = bytes.fromhex('2400020f0000011100000000000000000000000000000000000000000000000000002020')
        self.startSecondCommunication_cmd = bytes.fromhex('07000201000008')
        self.shutdown_cmd = bytes.fromhex('2400020f0000001180000000000000000000000000000000000000000000000000002020')
        self.turnOnLaser_cmd = bytes.fromhex('0a000203006100000101')
        self.turnOffLaser_cmd = bytes.fromhex('0a000203006100000100')
        self.requestMeasure_cmd = bytes.fromhex('08000202006d0000')
        self.requestTemp_cmd = bytes.fromhex('0800020200990000')

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

        self.read_distance_service = rospy.Service('~read_distance', GetDistance, self.read_distance_service_cb)
        self.read_distance_service = rospy.Service('~start_communication', Trigger, self.start_communication_cb)
        self.read_distance_service = rospy.Service('~shutdown_communication', Trigger, self.shutdown_communication_cb)
        self.read_distance_service = rospy.Service('~turn_off_laser', Trigger, self.turn_off_laser_cb)
        self.read_distance_service = rospy.Service('~turn_on_laser', Trigger, self.turn_on_laser_cb)

        #self.command_status_pub = rospy.Publisher(
        #    '~status', CommandManagerStatus, queue_size=1)
        
        return 0
    
    def ready_state(self):

        if self.communication_initialized == False:
            self.communication_initialized = self.initialize_communication()
            self.read()


        return
    
    def shutdown(self):
        # Turn off laser
        rospy.loginfo("%s::shutdown: Turns off laser" % self._node_name)
        self.write(self.turnOffLaser_cmd)

        rospy.loginfo("%s::shutdown: Sends shutdown communication message" % self._node_name)
        self.write(self.shutdown_cmd)
        rospy.sleep(1.0)

        self.serial_device.close()
        
        RComponent.shutdown(self)
    
    def write(self, data):
        bytes_written = self.serial_device.write(data)
        #rospy.loginfo("%s::write: bytes_written: %i", self._node_name, bytes_written)
        self.serial_device.flush()
        return bytes_written
    
    def read(self):
        try:
            data_read = self.serial_device.readline().hex()
            for c in self.serial_device.readline().hex():
                data_read=data_read + c
        except SerialException as e:
            rospy.logwarn(e)
            return

        return data_read

    def initialize_communication(self):
        self.write(self.startFirstCommunication_cmd)
        rospy.loginfo("%s::initialize_communication: First communication message sent", self._node_name)
        rospy.sleep(0.5)


        rospy.loginfo("%s::initialize_communication: Sends second communication message", self._node_name)
        self.write(self.startSecondCommunication_cmd)
        rospy.sleep(0.5)

        # Turn on laser
        #rospy.loginfo("Turns on laser")
        #self.write(self.turnOnLaser_cmd)
        #rospy.sleep(0.5)

        return True
    
    def read_distance_service_cb(self, msg):
        self.turn_on_laser_cb(TriggerRequest())
        self.serial_device.flush()
        # Reads datas
        num_reads = self.reads_per_request
        if msg.readings > 0:
            num_reads = msg.readings
        measurements = []
        measured_quality = []
        measured_temp = []
        rospy.loginfo("%s::read_distance_service_cb: Reads distance from laser", self._node_name)
        for x in range(0,num_reads):
            self.write(self.requestMeasure_cmd)
            distance = self.read()
            #print(distance)
            quality=distance[16:20]
            #print(quality)
            distance=distance[12:16]
            #print(distance)
            measurements.append(float(int(distance,16))/10.0)
            measured_quality.append(float(int(quality,16))/100.0)
            self.write(self.requestTemp_cmd)
            temperature = self.read()
            #print(temperature)
            temperature=temperature[12:14]
            #print(temperature)
            measured_temp.append(float(int(temperature,16)))
            rospy.loginfo("%s::read_distance_service_cb: Distance read (in 1/10 mm): %.2f \t Quality: %.2f \t Temp: %.2f", self._node_name, measurements[x], measured_quality[x],measured_temp[x])

        #average_distance = statistics.mean(measurements)
        #average_quality = statistics.mean(measured_quality)
        #average_temperature = statistics.mean(measured_temp)
        average_distance = statistics.median(measurements)
        average_quality = statistics.median(measured_quality)
        average_temperature = statistics.median(measured_temp)
        std_dev = statistics.stdev(measurements)
        rospy.loginfo("%s::read_distance_service_cb: Mean -> %.4f", self._node_name, average_distance)
        rospy.loginfo("%s::read_distance_service_cb: Standard deviation -> %.4f", self._node_name, std_dev)
        
        self.turn_off_laser_cb(TriggerRequest())

        response = GetDistanceResponse()
        response.success = True
        response.dist = measurements
        response.level = measured_quality
        response.temp = measured_temp
        response.distance = average_distance
        response.std_dev = std_dev
        response.signal_level = average_quality
        response.sensor_temp = average_temperature
        response.message = "Distance correctly read"
        return response

    def shutdown_communication_cb(self, msg):
        self.serial_device.flush()

        rospy.loginfo("%s::shutdown_communication_cb: Sends shutdown communication message", self._node_name)
        self.write(self.shutdown_cmd)
        rospy.sleep(1.0)

        return TriggerResponse()
        
    def turn_on_laser_cb(self, msg):
        self.serial_device.flush()
        self.write(self.turnOnLaser_cmd)
        self.read()
        return TriggerResponse()        

    def turn_off_laser_cb(self, msg):
        self.serial_device.flush()
        self.write(self.turnOffLaser_cmd)
        self.read()
        return TriggerResponse()
        
    def start_communication_cb(self, msg):
        self.serial_device.flush()

        rospy.loginfo("%s::start_communication_cb: Sends shutdown communication message", self._node_name)
        self.write(self.startFirstCommunication_cmd)
        rospy.sleep(0.5)

        self.write(self.startSecondCommunication_cmd)
        rospy.sleep(0.5)
        return TriggerResponse()
