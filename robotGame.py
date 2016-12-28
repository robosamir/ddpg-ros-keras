#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header
from tf import TransformListener
import math

class robotGame():
    def __init__(self):
        #init code
        rospy.init_node("robotGame")
        self.currentDist = 1
        self.previousDist = 1
        self.reached = False
        self.tf = TransformListener()

        self.left_joint_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'yumi_joint_7_l', 'gripper_l_joint']
        self.right_joint_names = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r', 'yumi_joint_7_r', 'gripper_r_joint']
        self.left_positions = [-2.01081820427463881, 1.4283937236421274, -1.3593418228836045, -0.19315625641494183, 1.7016501799872579, 0.6573540231496411, 3.404315594906305, 0.0]
        self.right_positions = [0.01081820427463881, 2.4283937236421274, 0.3593418228836045, -0.19315625641494183, 1.7016501799872579, 0.6573540231496411, 3.404315594906305, 1.8145107750466565]
        self.rjv = []
        self.ljv = []

        self.pub = rospy.Publisher('my_joint_states', JointState, queue_size=1) 
        self.js = JointState()
        self.js.header = Header()
        self.js.name = self.left_joint_names + self.right_joint_names
        self.js.velocity = []
        self.js.effort = []
        self.sub = rospy.Subscriber('/joint_states', JointState, self.jsCB)
        self.destPos = np.random.uniform(0,0.25, size =(3))
        self.reset()

    def jsCB(self,msg):
        temp_dict = dict(zip(msg.name, msg.position))
        self.rjv = [temp_dict[x] for x in self.right_joint_names]
        self.ljv = [temp_dict[x] for x in self.left_joint_names]
        self.js.position = self.left_positions + self.right_positions
        self.js.header.stamp = rospy.Time.now()
        self.pub.publish(self.js)

    def getCurrentJointValues(self):
        return self.rjv

    def getCurrentPose(self):
        self.tf.waitForTransform("/world","/gripper_r_finger_r",rospy.Time(),rospy.Duration(10))
        t = self.tf.getLatestCommonTime("/world", "/gripper_r_finger_r") 
        position, quaternion = self.tf.lookupTransform("/world","/gripper_r_finger_r",t)
        return [position[0],position[1],position[2]]

    def setJointValues(self,tjv):
        self.right_positions = tjv
        self.right_positions[-1] = 0 #close gripper
        rospy.sleep(0.20)
        return True

    def getDist(self):
        position = self.getCurrentPose()
        currentPos = np.array((position[0],position[1],position[2]))
        return np.linalg.norm(currentPos-self.destPos)

    def reset(self):
        self.setJointValues(np.random.uniform(-1,1, size=(8)).tolist())
        #print self.destPos
        self.destPos = np.random.uniform(0,0.25, size =(3))
        #print self.destPos
        tjv = self.getCurrentJointValues()
        positions = self.getCurrentPose()
        return tjv+positions+self.destPos.tolist()
 
    def step(self,vals):
        done = False
        prevDist = self.getDist()
        tjv = [x + y for x,y in zip(vals.flatten().tolist(),self.getCurrentJointValues())]
        status = self.setJointValues(tjv)
        curDist = self.getDist()
        reward = -curDist - 0.00*np.linalg.norm(vals) - 0.5*math.log10(curDist) 
        print  self.destPos, -curDist - 0.5*math.log10(curDist) ,-curDist, np.linalg.norm(vals)
        if curDist < 0.01:
            reward +=10 
            done = True
        tjv = self.getCurrentJointValues()
        positions = self.getCurrentPose()
        return [tjv+positions+self.destPos.tolist(), reward, done]

    def done(self):
        self.sub.unregister()
        rospy.signal_shutdown("done")

if __name__ == "__main__":
            r = robotGame()
            print r.getCurrentJointValues()
            print r.getCurrentPose()
            r.reset()
            print r.getCurrentJointValues()
            print r.getCurrentPose()




