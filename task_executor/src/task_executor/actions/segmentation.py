#!/usr/bin/env python3
import rospy

from stretch_fetch_grasp_bridge.srv import StretchSegmentation, StretchSegmentationRequest, StretchSegmentationResponse
from stretch_fetch_grasp_bridge.srv import StretchGraspPosev2, StretchGraspPosev2Request, StretchGraspPosev2Response
from rerail_stretchit_manipulation.srv import RerailManip, RerailManipRequest, RerailManipResponse
from task_executor.abstract_step import AbstractStep

class SegmentationAction(AbstractStep):
    SEGMENTATION_ACTION_SERVER = '/stretch_segmentation/segment_objects'
    def init(self, name):
        self._stopped = False
        rospy.loginfo("Waiting for service "+ SegmentationAction.SEGMENTATION_ACTION_SERVER)
        rospy.wait_for_service(SegmentationAction.SEGMENTATION_ACTION_SERVER)
        self.get_segmentation = rospy.ServiceProxy(SegmentationAction.SEGMENTATION_ACTION_SERVER, StretchSegmentation)

    def run(self, object = "can"):
        req = StretchSegmentationRequest()
        req.object_name = object
        resp = self.get_segmentation(req)
        segmented_object = resp.segmented_point_cloud
        yield self.set_running()

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self.get_segmentation.resolved_name,
                segmented_object=segmented_object
            )
        elif not resp.success:
            yield self.set_aborted(
                action=self.name,
                srv=self.get_segmentation.resolved_name,
                segmented_object=segmented_object
            )
        else:
            yield self.set_succeeded(segmented_object=segmented_object)

    def stop(self):
        self._stopped = True
        # self.get_segmentation.cancel_goal()
        # self.notify_action_cancel(SegmentationAction.SEGMENTATION_ACTION_SERVER)