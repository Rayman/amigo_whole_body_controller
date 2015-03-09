#!/usr/bin/env python

'''
Send a Cartesian Motion Objective
'''

import roslib; roslib.load_manifest('amigo_whole_body_controller')
import rospy
import actionlib

from amigo_whole_body_controller.msg import *
from arm_navigation_msgs.msg import *
from tf.transformations import quaternion_from_euler

def euler_z_to_orientation_goal(orientation=None):
    roll  = orientation[0] if orientation else 0
    pitch = orientation[0] if orientation else 0
    yaw   = orientation[0] if orientation else 0

    q = geometry_msgs.msg.Quaternion(*quaternion_from_euler(roll, pitch, yaw))

    # header,link_name,type,orientation,
    # absolute_roll_tolerance,absolute_pitch_tolerance,absolute_yaw_tolerance
    # weight

    return OrientationConstraint(
        orientation=q)

class CartesianMotionObjective():

    stiffnessfactor = 4.0

    def __init__(self):
        self.force = (
            70.0 * self.stiffnessfactor,
            70.0 * self.stiffnessfactor,
            50.0 * self.stiffnessfactor)
        self.torque = (
            5.0 * self.stiffnessfactor,
            5.0 * self.stiffnessfactor,
            5.0 * self.stiffnessfactor)

        self.action_client = actionlib.SimpleActionClient("amigo/add_motion_objective", ArmTaskAction)
        self.action_client.feedback_cb = self.feedback_cb

        self.action_client.wait_for_server()
        rospy.loginfo("Connected to action server")

    def build_goal(self, link_name, position, orientation, frame_id):
        self.position    = position
        orientation_goal = euler_z_to_orientation_goal(orientation)

        force = self.force
        torque = self.torque if orientation else (0, 0, 0)

        position_constraint = PositionConstraint(
            header=std_msgs.msg.Header(frame_id=frame_id),
            link_name=link_name,
            position=geometry_msgs.msg.Point(*self.position))

        stiffness = geometry_msgs.msg.Wrench(
            force=geometry_msgs.msg.Vector3(*force),
            torque=geometry_msgs.msg.Vector3(*torque))

        goal = ArmTaskGoal(
            goal_type='grasp',
            position_constraint=position_constraint,
            orientation_constraint=orientation_goal,
            stiffness=stiffness)
        return goal

    def send_goal(self, goal):
        rospy.loginfo("Sending goal...")
        print goal

        rospy.loginfo("Waiting for result...")
        self.action_client.send_goal(goal,
            done_cb=None, active_cb=None, feedback_cb=self.feedback_cb)

    def wait_for_result(self):
        finished = False
        try:
            finished = self.action_client.wait_for_result()
        except KeyboardInterrupt:
            print "Canceling goal..."
            self.action_client.cancel_goal()
            self.action_client.wait_for_result(rospy.Duration(0.5))
        if not finished:
            return

        rospy.loginfo("Result!")
        result = self.action_client.get_result()
        print result

    last_feedback = None
    def feedback_cb(self, feedback):
        code = feedback.status_code.status
        if code != self.last_feedback:
            print 'feedback!!!', code
            self.last_feedback = code
