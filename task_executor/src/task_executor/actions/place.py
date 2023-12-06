#!/usr/bin/env python3
import rospy

from task_executor.abstract_step import AbstractStep
from std_srvs.srv import Trigger, TriggerRequest

class Place(AbstractStep):
    PLACE_OBJECT_ACTION_SERVICE = 'execute_delivery_task'
    def init(self, name):
        # Create the clients
        rospy.wait_for_service(Place.PLACE_OBJECT_ACTION_SERVICE)
        self.place = rospy.ServiceProxy(Place.PLACE_OBJECT_ACTION_SERVICE, Trigger)
        self.place_req = TriggerRequest()

    def run(self, height):
        # TO-DO we are not using the height parameter right now
        # getting segmented cloud
        result = self.place(self.place_req)
        if result.success == True:
            yield self.set_succeeded()
        

    def stop(self):
        print("completed place movement")