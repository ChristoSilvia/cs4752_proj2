#!/usr/bin/env python

import numpy as np

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics

class MotorController():
    def __init__(self, limb_name):
        rospy.init_node('robot_interface')
        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)
        self.joint_names = self.limb.joint_names()
        self.kp_coefficient = 0.004
        self.dt = 0.005

    def get_joint_velocities(self, workspace_velocity_and_w):
        jacobian_pinv = self.limb_kin.jacobian_pseudo_inverse()
        return np.dot(jacobian_pinv, workspace_velocity_and_w)

    def moveto_proportional(self, final_position, speed):
		self.limb.set_command_timeout(3*self.dt)

		kp = self.kp_coefficient * speed
        initial_position = self.get_position()
		total_distance = np.linalg.norm(initial_position - final_position)
		total_time = total_distance / speed
        velocity = speed * (final_position - initial_position) / total_distance

        for t in np.arange(0, total_time, dt):
			t_start = rospy.get_time()
			current_position = self.get_position()
			desired_position = initial_position + (t/total_time) * (final_position - initial_position)
			current_error = current_position - desired_position

			velocity -= kp * current_error

			velocity_and_w = np.zeros((6,1))
			velocity_and_w[0:3,0] = velocity

			self.limb.set_joint_velocities(
				self.make_velocity_dict(
					self.joint_velocities(velocity_and_w)))

			extra_time = dt - (rospy.get_time() - t_start)
			rospy.sleep(extra_time)

        self.limb.exit_control_mode()


    def make_velocity_dict(self, joint_velocity_vector):
        velocity_dict = {}
        for joint_velocity, joint_name in zip(joint_velocity_vector, self.joint_names):
            velocity_dict[joint_name] = joint_velocity
        return velocity_dict
        
    def get_position(self):
		return np.array(self.limb.endpoint_pose()['position'])

if __name__ == '__main__':
	try:
		MotorController('left')
	except rospy.ROSInterruptException:
		pass
