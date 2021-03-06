from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!

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


    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

    ### Your IK code here 
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    R_z = Matrix([[ cos(pi), -sin(pi), 0, 0],
                    [ sin(pi),  cos(pi), 0, 0],
                    [       0,        0, 1, 0],
                    [       0,        0, 0, 1]])

    R_y = Matrix([[  cos(-pi/2), 0, sin(-pi/2), 0],
                    [           0, 1,          0, 0],
                    [ -sin(-pi/2), 0, cos(-pi/2), 0],
                    [           0, 0,          0, 1]])
    
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
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    T0_4 = T0_1 * T1_2 * T2_3 * T3_4
    T0_4 = T0_4.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    T0_G = T0_G * R_corr
    T0_G = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0],WC[1],WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [T0_G[0,3],T0_G[1,3],T0_G[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
