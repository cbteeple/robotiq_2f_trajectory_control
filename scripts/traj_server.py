#!/usr/bin/env python

import rospy
import actionlib
from scipy.interpolate import interp1d


#Import the specific messages that we created in our tutorials folder.
import robotiq_trajectory_control.msg as msg

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq




class TrajAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.TrajectoryFeedback()
    _result = msg.TrajectoryResult()

    def __init__(self, name, comms_obj=None, controller_rate=50):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.comms=comms_obj
        self.controller_rate=controller_rate

        #action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.command_client = actionlib.SimpleActionClient(self._action_name+'/command_robotiq_action', CommandRobotiqGripperAction)

        # Start an actionlib server
        self._as = actionlib.SimpleActionServer(self._action_name+'/robotiq_trajectory', msg.TrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def execute_cb(self, goal):
        traj= goal.trajectory.points

        # Put all of the setpoints and durations into numpy arrays for later use
        self.traj_points=[]
        self.traj_times=[]
        for point in traj:
            self.traj_points.append(point.pos)
            self.traj_times.append(point.time_from_start.to_sec())

        print(self.traj_points)
        print(self.traj_times)

        traj_interp = interp1d(self.traj_times,self.traj_points, axis=0)

        

        # helper variables
        r = rospy.Rate(self.controller_rate)
        success = False
        
        # Initiatilize the feedback
        self._feedback.current_time = 0
        self._feedback.success = True

        # Send the pressure trajectory in real time
        start_time=rospy.Time.now()
        curr_time = rospy.Duration(0.0)

        idx=0

        cur_pt_idx = 0

        while curr_time.to_sec() < self.traj_times[-1] and not rospy.is_shutdown():
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._feedback.success = False
                break
            else:
                # Interpolate point to send from the trajectory given
                curr_time = rospy.Time.now()-start_time

                if curr_time.to_sec() >= self.traj_times[-1] : 
                    break

                curr_time_sec = curr_time.to_sec()
                curr_time_pt = traj[cur_pt_idx].time_from_start.to_sec()

                if curr_time_pt >= curr_time_sec:
                    Robotiq.goto(self.command_client,
                                    pos   = traj[cur_pt_idx].pos,
                                    speed = traj[cur_pt_idx].speed,
                                    force = traj[cur_pt_idx].force)

                    cur_pt_idx+=1 # Update the index to move on to the next point



                # Update the server
                self._feedback.current_time  = curr_time
                self._as.publish_feedback(self._feedback)

                r.sleep()
                idx += 1


        Robotiq.goto(self.command_client,
                        pos   = traj[-1].pos,
                        speed = traj[-1].speed,
                        force = traj[-1].force)



        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            #rospy.loginfo("End: %s"%(goal.command))




    def shutdown(self):
        pass
        #self.comms.shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        print("TRAJECTORY SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
        server = TrajAction(rospy.get_namespace())
        print("TRAJECTORY SERVER: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    