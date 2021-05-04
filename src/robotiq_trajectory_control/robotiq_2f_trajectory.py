#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from robotiq_trajectory_control.msg import *

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


import time
import sys
import os
import numbers
import numpy as np
import copy
import yaml
import threading


class trajSender:
    def __init__(self, speed_factor = 1.0, name = "robotiq_2f_control"):
        self._name = ''

        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.command_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

        self.traj_client = actionlib.SimpleActionClient(self._name+'/robotiq_trajectory', TrajectoryAction)
        self.command_client.wait_for_server()
        self.traj_client.wait_for_server()

        if speed_factor <=0.0:
            speed_factor = 1.0

        self.speed_multiplier = 1./speed_factor
        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        

    # build the whole trajectory
    def build_traj(self, traj):
        traj_goal = TrajectoryGoal()
        traj_goal.trajectory = Trajectory()

        for entry in traj:
            traj_goal.trajectory.points.append(TrajectoryPoint(
                        pos         = entry[1],
                        speed       = entry[2],
                        force       = entry[3],
                        time_from_start = self.speed_multiplier*rospy.Duration(entry[0])))

        return traj_goal



    def go_to_start(self, traj_goal, reset_time, blocking=True):
        if traj_goal is None:
            return False
        self.start_pos = traj_goal[0][1]

        goal_tmp = TrajectoryGoal()
        goal_tmp.trajectory = Trajectory()

        goal_tmp.trajectory.points.append(TrajectoryPoint(pos=self.start_pos, speed=0.10, force=0.0, time_from_start=rospy.Duration(traj_goal[0][0])))
        goal_tmp.trajectory.points.append(TrajectoryPoint(pos=self.start_pos, speed=0.10, force=0.0, time_from_start=rospy.Duration(traj_goal[0][0])))

        self.execute_traj(goal_tmp, blocking)





    def execute_traj(self, traj_goal, blocking=True):
        print(traj_goal)
        #try:
        self.traj_client.send_goal(traj_goal)

        print('Goal Sent')

        if blocking:
            self.traj_client.wait_for_result()
        else:
            return self.traj_client

        #except KeyboardInterrupt:
        #    self.traj_client.cancel_goal()
        #except:
        #    raise


            

    def send_command(self, command, args=[]):
        # Validate inputs
        if not isinstance(command, str):
            raise ValueError('CONFIG: Command must be a string')

        if isinstance(args, dict):
            pass
        elif isinstance(args, numbers.Number):
            args=[args]
        else:
            raise ValueError('CONFIG: Args must be a list, tuple, or number')

        if self.DEBUG:
            print(command, args)

        # Send commands to the commader node and wait for things to be taken care of

        command = command.lower()
        if command == 'emergency_release':
            Robotiq.emergency_release(self.command_client)
        elif command == 'open':
            Robotiq.open(self.command_client, **args)
        elif command == 'close':
            Robotiq.close(self.command_client, **args)
        elif command == 'stop':
            Robotiq.stop(self.command_client, **args)
        elif command == 'goto':
            Robotiq.goto(self.command_client, **args)


    def shutdown(self, reset=None):
        print('HAND CONTROLLER: Cancel current goals')
        self.traj_client.cancel_all_goals()
           
        if reset is not None:
            print('HAND CONTROLLER: Setting resting position')

            if reset == 'resting':
                out_press= copy.deepcopy(self.start_pos)
                out_press = out_press[0]
                print(out_press)
                self.send_command("goto",{'pos':out_press})
            else:
                self.send_command("goto",reset)

        self.command_client.cancel_all_goals()
        




if __name__ == '__main__':
    pass