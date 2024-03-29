#!/usr/bin/env python3
# The wait action

import rospy

from task_executor.abstract_step import AbstractStep


# The action class

class WaitAction(AbstractStep):
    """Wait for a specified duration"""

    def init(self, name):
        self.name = name
        self._stopped = False

    def run(self, duration):
        """
        The run function for this step

        Args:
            duration (float) : the duration, in secs, to wait for

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Waiting for {}s".format(self.name, duration))

        self._stopped = False
        start_time = rospy.Time.now()
        while rospy.Time.now() <= start_time + rospy.Duration(duration):
            if self._stopped:
                break

            yield self.set_running()

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=duration
            )
        else:  # Succeeded
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True