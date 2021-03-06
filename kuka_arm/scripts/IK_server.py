#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### FK code
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # for rotations
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        
        # Create Modified DH parameters
        s = {alpha0: 0,     a0: 0,      d1: 0.75,
             alpha1: -pi/2, a1: 0.35,   d2: 0,      q2: q2 - pi/2,
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6:0,
             alpha6:0,      a6: 0,      d7: 0.303,  q7:0}
             
        # Define Modified DH Transformation matrix
        def TF_Matrix(q, d, a, alpha):
            return Matrix([[            cos(q),           -sin(q),           0,             a],
                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                 0,                 0,           0,             1]])
        
        # Create individual transformation matrices
        T0_1 = TF_Matrix(q1, d1, a0, alpha0).subs(s)
        T1_2 = TF_Matrix(q2, d2, a1, alpha1).subs(s)
        T2_3 = TF_Matrix(q3, d3, a2, alpha2).subs(s)
        T3_4 = TF_Matrix(q4, d4, a3, alpha3).subs(s)
        T4_5 = TF_Matrix(q5, d5, a4, alpha4).subs(s)
        T5_6 = TF_Matrix(q6, d6, a5, alpha5).subs(s)
        T6_G = TF_Matrix(q7, d7, a6, alpha6).subs(s)
        
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # Create full transformation matrix from end position
        r, p, y = symbols('r p y')

        Rot_x = Matrix([[       1,       0,       0],
                        [       0,  cos(r), -sin(r)],
                        [       0,  sin(r),  cos(r)]])

        Rot_y = Matrix([[  cos(p),       0,  sin(p)],
                        [       0,       1,       0],
                        [ -sin(p),       0,  cos(p)]])

        Rot_z = Matrix([[  cos(y), -sin(y),       0],
                        [  sin(y),  cos(y),       0],
                        [       0,       0,       1]])

        Rot_G = Rot_z * Rot_y * Rot_x


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_z = Rot_z.subs({y: pi})
            R_y = Rot_y.subs({p: -pi/2})
            
            R_corr = R_z * R_y
            Rot_G_corr = Rot_G * R_corr[:3,:3]
            Rot_G_corr = Rot_G_corr.subs({r:roll, p:pitch, y:yaw})
            
            # Calculate joint angles using Geometric IK method

            # Calculate position of wrist center as:
            # "position of end effector" - "distance between wrist and end effoctor" * "local z-axis"
            G = Matrix([[px],
                       [py],
                       [pz]])
            WC = G - 0.303 * Rot_G_corr[:,2]
            
            # theta1 can be obtained with a top projection
            theta1 = atan2(WC[1], WC[0])

            # we calculate theta2 and theta3 by projecting to local plane, refer to figure in write_up
            # distance between joint 3 and wrist center
            a = sqrt(pow(1.5, 2) + pow(0.054, 2))
            # distance between joint 2 and wrist center
            b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            # distance between joint 2 and joint 3
            c = 1.25

            # use cosine law to find angle values
            ang_a = acos((b * b + c * c - a * a) / (2 * b * c))
            ang_b = acos((a * a + c * c - b * b) / (2 * a * c))
            ang_c = acos((a * a + b * b - c * c) / (2 * a * b))

            # theta2 can be calculated from sum of angles at joint 2
            theta2 = pi / 2 - ang_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)

            # theta3 can be calculated from sum of angles at joint 3
            theta3_init = pi/2 + atan2(0.054, 1.5) # initial angle at joint 3 (see figure in write_up)
            theta3 = pi - ang_b - theta3_init

            # calculate transformation from base frame to frame at joint 3
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # calculate transformation between joint 3 and end effector
            R3_G = R0_3.transpose() * Rot_G_corr

            # the easiest is to calculate the symbolic form of R3_G from joint angles with sympy
            # then we can easily find formulas to calculate the missing theta values (refer to writeup)
            theta4 = atan2(R3_G[2,2], -R3_G[0,2])
            theta5 = atan2(sqrt(R3_G[0,2]*R3_G[0,2] + R3_G[2,2]*R3_G[2,2]), R3_G[1,2])
            theta6 = atan2(-R3_G[1,1], R3_G[1,0])
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
