#!/usr/bin/env python3
import rospy

from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from stretch_fetch_grasp_bridge.srv import StretchGraspPosev2, StretchGraspPosev2Request, StretchGraspPosev2Response
from rerail_stretchit_manipulation.srv import RerailManip, RerailManipRequest, RerailManipResponse
from task_executor.abstract_step import AbstractStep

class GraspAction(AbstractStep):
    GRASP_ACTION_SERVER = '/stretch_grasp_pose_suggester'
    def init(self, name):
        self._stopped = False
        rospy.loginfo("Waiting for service "+GraspAction.GRASP_ACTION_SERVER)
        rospy.wait_for_service(GraspAction.GRASP_ACTION_SERVER)
        self.get_grasp = rospy.ServiceProxy(GraspAction.GRASP_ACTION_SERVER,StretchGraspPosev2)

    def run(self, segmented_object):
        req = StretchGraspPosev2Request()
        req.point_cloud = segmented_object
        rospy.loginfo("sending grasp request!")
        grasp = self.get_grasp(req)

        yield self.set_running()

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self.get_grasp.resolved_name,
                grasp=grasp
            )
        elif not grasp.success:
            yield self.set_aborted(
                action=self.name,
                srv=self.get_grasp.resolved_name,
                grasp=grasp
            )
        else:
            yield self.set_succeeded(grasp=grasp)

        

    def stop(self):
        self._stopped = True
        # self.get_grasp.cancel_goal()
        # self.notify_action_cancel(GraspAction.GRASP_ACTION_SERVER)