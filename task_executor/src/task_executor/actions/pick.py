#!/usr/bin/env python3
import rospy

from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from stretch_fetch_grasp_bridge.srv import StretchGraspPosev2, StretchGraspPosev2Request, StretchGraspPosev2Response
from rerail_stretchit_manipulation.srv import RerailManip, RerailManipRequest, RerailManipResponse
from task_executor.abstract_step import AbstractStep

class Pick(AbstractStep):
    MANIPULATE_ACTION_SERVER = '/rerail_stretchit_manipulation/manipulate'
    def init(self, name):
        self._stopped = False
        rospy.loginfo("Waiting for service "+Pick.MANIPULATE_ACTION_SERVER)
        rospy.wait_for_service(Pick.MANIPULATE_ACTION_SERVER)
        self.exec_manip = rospy.ServiceProxy(Pick.MANIPULATE_ACTION_SERVER, RerailManip)

    def run(self, grasp):
        # Manipulation Task
        req = RerailManipRequest()
        req.target_pose = grasp.grasp_pose.pose
        rospy.loginfo("sending manipulation request!")
        manip = self.exec_manip(req)

        yield self.set_running()

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self.exec_manip.resolved_name,
                goal=grasp
            )
        elif not manip.success:
            yield self.set_aborted(
                action=self.name,
                srv=self.exec_manip.resolved_name,
                goal=grasp
            )
        else:
            yield self.set_succeeded()
        


    def stop(self):
        self._stopped = True
        # self.exec_manip.cancel_goal()

        # self.notify_action_cancel(Pick.MANIPULATE_ACTION_SERVER)