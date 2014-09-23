#!/usr/bin/env python
__author__ = 'mandeep'
import rospy
import pickle
from dynamixel_controllers.srv import TorqueEnable
from os.path import isfile
import actionlib
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import String

class zeno_walk:
    numMotors=12
    names = [
        'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
        'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll'
    ]
    motorNamesAndCat={
        'r_hip_yaw':'r', 'r_hip_roll':'r', 'r_hip_pitch':'r', 'r_knee_pitch':'r', 'r_ankle_pitch':'r', 'r_ankle_roll':'r',
        'l_hip_yaw':'l', 'l_hip_roll':'l', 'l_hip_pitch':'l', 'l_knee_pitch':'l', 'l_ankle_pitch':'l', 'l_ankle_roll':'l'
    }#as in yaml array of tuple name,left or right
    nameAndIdIndex={
        'r_hip_yaw':1,'r_hip_roll':2, 'r_hip_pitch':3, 'r_knee_pitch':4, 'r_ankle_pitch':5, 'r_ankle_roll':6,
        'l_hip_yaw':7, 'l_hip_roll':8, 'l_hip_pitch':9, 'l_knee_pitch':10, 'l_ankle_pitch':11, 'l_ankle_roll':12
    }
    motorTrajectoryTopics={
        'l':'/left_leg_controller/follow_joint_trajectory',
        'r':'/right_leg_controller/follow_joint_trajectory'
    }#dictionary category:TrajectoryTopic

    totalFrames=1
    currentFrame=1
    TrajectoryInfo={
        'r_hip_yaw':[], 'r_hip_roll':[], 'r_hip_pitch':[], 'r_knee_pitch':[], 'r_ankle_pitch':[], 'r_ankle_roll':[],
        'l_hip_yaw':[], 'l_hip_roll':[], 'l_hip_pitch':[], 'l_knee_pitch':[], 'l_ankle_pitch':[], 'l_ankle_roll':[]
    }
    TimeDelayFromPrevious=[]

    animation_map={
        'walk_forward':'walk_forward.dyn'
    }

    def __init__(self):
        rospy.init_node('zeno_walk_node')
        self.anim_dir=rospy.get_param('~animations_directory','')
        self.setAllTorque(True)
        self.play(self.animation_map['walk_forward'],1)
        rospy.Subscriber('/zeno_walk_command',String,self.callback)

    def callback(self,data):
        if data.data in self.names:
            self.play(self.animation_map[data.data],0)

    def setAllTorque(self,stt):
        for name in self.names:
            self.setMotorTorque(name,stt)

    def setMotorTorque(self,name,stt):
        service_name = name + '_controller/torque_enable'
        torque_enable_srv = rospy.ServiceProxy(service_name, TorqueEnable)
        rospy.loginfo("Waiting for {0} service".format(service_name))
        rospy.wait_for_service(service_name)
        rospy.loginfo("{0} service found".format(service_name))
        torque_enable_srv(stt)

    def play(self,fileName,fix_frame=0):
        filename=self.anim_dir+fileName
        print("\nOpening File:"+filename+"\n")
        if isfile(filename):
            self.animation=pickle.load(open(filename,"rb"))
            self.TimeDelayFromPrevious=self.animation[0]
            self.TrajectoryInfo=self.animation[1]
        else:
            return

        startFrame=1
        stopFrame=len(self.TimeDelayFromPrevious)
        if fix_frame>0 and fix_frame>=startFrame and fix_frame<=stopFrame:
            startFrame=fix_frame
            stopFrame=fix_frame

        if startFrame>stopFrame or startFrame<1:
            print("\nwrong range in play\n")
            return

        trajectories=[]
        for cat in self.motorTrajectoryTopics:
            jointTraj=JointTrajectory()
            jts_names=[]
            for name in self.names:
                #if self.motorNamesAndCat[name]==cat and name!='l_hip_yaw':
                if self.motorNamesAndCat[name]==cat:
                    jointTraj.joint_names.append(name+"_joint")
                    jts_names.append(name)

            fcount=stopFrame-startFrame+1
            totalDelay=0.0
            for n in range(0,fcount):
                pos_arr=JointTrajectoryPoint()
                readLoc=n+startFrame
                delaySecs=float(self.TimeDelayFromPrevious[readLoc-1])/1000.0
                totalDelay=totalDelay+delaySecs
                pos_arr.time_from_start=rospy.Duration().from_sec(totalDelay)
                for name in jts_names:
                    pos_arr.positions.append(self.TrajectoryInfo[name][readLoc-1])
                    print("\n "+name+": "+str(self.TrajectoryInfo[name][readLoc-1]))
                    velocity=1.0#2*pi rad/sec
                    if n!=0:
                        velocity=float(self.TrajectoryInfo[name][readLoc-1]-self.TrajectoryInfo[name][readLoc-2])/delaySecs
                    ##pos_arr.velocities.append(velocity)
                jointTraj.points.append(pos_arr)
            jointTraj.header.stamp=rospy.Time.now()
            trajectories.append(jointTraj)

        trajClients=[]
        mm=0
        for cat in self.motorTrajectoryTopics:
            trajClients.append(actionlib.SimpleActionClient(self.motorTrajectoryTopics[cat],FollowJointTrajectoryAction))
            trajClients[mm].wait_for_server()
            mm=mm+1
            #trajectories[nn] .. how to send this goal

        nn=0
        for traj in trajClients:
            goal=FollowJointTrajectoryGoal()
            #goal.trajectory.points=trajectories[nn].points
            goal.trajectory=trajectories[nn]
            traj.send_goal(goal)
            nn=nn+1

        print ("\nDone Sending Play Info...\n")

if __name__ == "__main__":
    zeno_walker=zeno_walk()
    rospy.spin()