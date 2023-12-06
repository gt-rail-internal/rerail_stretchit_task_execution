#!/usr/bin/env python3
import rospy

from task_executor.abstract_step import AbstractStep
from std_srvs.srv import Trigger, TriggerRequest

class SwitchModeAction(AbstractStep):
    def init(self, name):
        # Create the clients
        rospy.wait_for_service('switch_to_position_mode')
        self.position_mode = rospy.ServiceProxy('switch_to_position_mode', Trigger)
        self.position_mode_req = TriggerRequest()

        rospy.wait_for_service('switch_to_navigation_mode')
        self.navigation_mode = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)
        self.navigation_mode_req = TriggerRequest()

    def run(self, mode):
        # getting segmented cloud
        if(mode == "position"):
            result = self.position_mode(self.position_mode_req)
        elif(mode == "navigation"):
            result = self.navigation_mode(self.navigation_mode_req)
        else:
            result = {"success": False}
            yield self.set_aborted(
                action=self.name,
                goal=mode,
                exception="invalid mode"   
            )
            print("invalid mode") 
        

        
        if result.success == True:
            yield self.set_succeeded()
        

    def stop(self):
        print("finished switching to nav")
        pass