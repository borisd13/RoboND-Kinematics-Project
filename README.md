## Project: Kinematics Pick & Place

The present project illustrates my solution to the [Udacity Pick and Place project for RoboND](https://github.com/udacity/RoboND-Kinematics-Project).

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Using the urdf file, we can plot the position of each joint.

![alt text](images/DH_graph.png)

We can then use the coordinates of the joints to derive DH parameters.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

We verify our values are correct by running the forward_kinematics demo.

![alt text](images/test_forward.png)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The individual transformation matrices are easily derived from the DH parameters by replacing them in the general DH transformation matrix:

![alt text](images/RH_parameters.png)

We obtain the full transformation from fixed base to gripper by multplying all the individual matrices together: T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

We use trigonometry to find the angles on all different joints. We split the problem into 2:
- Position from the base frame to the wrist center
- Position from the wrist center to the end effector

We use the following steps:
- Find rotation matrix for end effector through its orientation (roll, pitch, yaw). It is just a multiplication of 3 rotations around different axes.
- Find position of wrist center through previous rotation matrix and position of end effector. We take the position of the end effector and use its local z-axis to calculate the position of the wrist center.
- use trigonometry to find the 3 first joint angles, leading to wrist center (see below sketch).
  - theta 1 can be obtained with a top projection
  - we calculate sides A, B, C through simple projections and use cosine law to calculate triangle angles
  - theta 2 is calculated from sum of angles at joint 2
  - theta 3 is calculated from sum of angles at joint 3, without forgetting the small initial angle when all joint angles are 0.
![alt text](images/angle_sketch.jpg)
- calculate transformation matrix from wrist center to end effector by using the inverse of transformation from the base frame to the wrist.
- we use sympy to have the analytical form of the previously calculated matrix (only the values we used are represented in this matrix)
![alt text](images/T3_G.png)
- we find ways to calculate theta 4, 5 and 6 from the analytical form (several possible alternatives exist):
  - theta 4 = atan2(-T3_G[3,3], T3_G[1,3]) = atan(sin(q4) / cos(q4)) = atan(tan(q4))
  - theta 5 = atan2(sqrt(T3_G[1,3]^2 + T3_G[3,3]^2), T3_G[2,3)
  - theta 6 = atan2(-T3_2[2,2], T3_2[2,1])

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code has been commented to explain the sequence of steps used. We follow the same logic described previously.

The following has been observed during the development of this project:
* Using the sympy library helps visualize the equations and see possible ways to find the angles of the joints. It also let us confirm which variable should be constant and the positive direction of angles, helping a lot in debugging.
* The difference of convention between urdf and DH parameters can make it difficult to find the correct values. In particular the correction matrix to rotate from local DH frame to URDF at end effector was not obvious. Displaying local frames can help.
* Doing a lot of sketches is key both for forward kinematics and even more for inverse kinematics! Performing tests and checking our values are correct joint after joint help in debugging any issue.

The code succeeds in picking correctly the objects and dropping them in the cylinder.
![alt text](images/drop_object.png)