#!/usr/bin/env python3
import rospy

from task_executor.abstract_step import AbstractStep
from std_srvs.srv import Trigger, TriggerRequest

class RepositionCam(AbstractStep):
    MOVE_CAMERA_ACTION_SERVICE = 'execute_camera_movement'
    def init(self, name):
        # Create the clients
        rospy.wait_for_service(RepositionCam.MOVE_CAMERA_ACTION_SERVICE)
        self.reposition_cam = rospy.ServiceProxy(RepositionCam.MOVE_CAMERA_ACTION_SERVICE, Trigger)
        self.reposition_cam_req = TriggerRequest()

    def run(self, pose):
        # getting segmented cloud
        result = self.reposition_cam(self.reposition_cam_req)
        if result.success == True:
            yield self.set_succeeded()
        

    def stop(self):
        print("completed cam movement")
        pass