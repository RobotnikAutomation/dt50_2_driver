#!/usr/bin/env python
import csv
import time

import rospy

from dt50_2_driver.srv import GetDistance, GetDistanceRequest
from rcomponent.rcomponent import *

from xmlrpc_server.rc_server import XMLRPCServer


class URBridge(XMLRPCServer):
    def __init__(self):
        XMLRPCServer.__init__(self)

    def ros_read_params(self):
        XMLRPCServer.ros_read_params(self)

        self.dt50_driver_ns = 'dt50_2_driver'
        self.dt50_driver_ns = rospy.get_param('~dt50_driver_ns', self.dt50_driver_ns)
        
        return 0

    def ros_setup(self):
        """Creates and inits ROS components"""
        self.read_data_service = rospy.ServiceProxy(self.dt50_driver_ns + '/read_distance', GetDistance)
        RComponent.ros_setup(self)
    
    
    def setup(self):
        self.server.register_function(self.read_data, 'read_data')
        
        XMLRPCServer.setup(self)

        return 0

    def ready_state(self):
        pass
    
    def read_data(self):
        data = self.read_data_service.call(GetDistanceRequest())
        f = open('/home/robot/test.csv', 'a+')
        writer = csv.writer(f)
        writer.writerow([data.distance, data.std_dev])
        f.close()

        return data.distance

