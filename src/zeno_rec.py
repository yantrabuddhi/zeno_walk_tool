__author__ = 'mandeep'

# -*- coding: utf-8 -*-
import rospy
import sys
from PyQt4 import QtCore, QtGui
from UI_zeno_rec import Ui_zeno_rec
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_msgs.msg import JointState as DynamixelJointState
from itertools import repeat


class StartQT4(QtGui.QMainWindow):
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
    # motorPositionTopics=[]#List
    # motorTorqueServices=[]#List
    # motorTorqueStates=[]#bool List
    motorTrajectoryTopics=[]#dictionary category:TrajectoryTopic

    totalFrames=1
    currentFrame=1
    TrajectoryInfo={
        'r_hip_yaw':[], 'r_hip_roll':[], 'r_hip_pitch':[], 'r_knee_pitch':[], 'r_ankle_pitch':[], 'r_ankle_roll':[],
        'l_hip_yaw':[], 'l_hip_roll':[], 'l_hip_pitch':[], 'l_knee_pitch':[], 'l_ankle_pitch':[], 'l_ankle_roll':[]
    }
    TimeDelayFromPrevious=[]

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_zeno_rec()
        self.ui.setupUi(self)

        QtCore.QObject.connect(self.ui.chkR1,QtCore.SIGNAL("toggled(bool)"),self.chkR1)
        QtCore.QObject.connect(self.ui.chkR2,QtCore.SIGNAL("toggled(bool)"),self.chkR2)
        QtCore.QObject.connect(self.ui.chkR3,QtCore.SIGNAL("toggled(bool)"),self.chkR3)
        QtCore.QObject.connect(self.ui.chkR4,QtCore.SIGNAL("toggled(bool)"),self.chkR4)
        QtCore.QObject.connect(self.ui.chkR5,QtCore.SIGNAL("toggled(bool)"),self.chkR5)
        QtCore.QObject.connect(self.ui.chkR6,QtCore.SIGNAL("toggled(bool)"),self.chkR6)
        QtCore.QObject.connect(self.ui.chkL7,QtCore.SIGNAL("toggled(bool)"),self.chkL7)
        QtCore.QObject.connect(self.ui.chkL8,QtCore.SIGNAL("toggled(bool)"),self.chkL8)
        QtCore.QObject.connect(self.ui.chkL9,QtCore.SIGNAL("toggled(bool)"),self.chkL9)
        QtCore.QObject.connect(self.ui.chkL10,QtCore.SIGNAL("toggled(bool)"),self.chkL10)
        QtCore.QObject.connect(self.ui.chkL11,QtCore.SIGNAL("toggled(bool)"),self.chkL11)
        QtCore.QObject.connect(self.ui.chkL12,QtCore.SIGNAL("toggled(bool)"),self.chkL12)
        QtCore.QObject.connect(self.ui.chkLA,QtCore.SIGNAL("toggled(bool)"),self.chkLA)
        QtCore.QObject.connect(self.ui.chkRA,QtCore.SIGNAL("toggled(bool)"),self.chkRA)
        QtCore.QObject.connect(self.ui.chkAllMotors,QtCore.SIGNAL("toggled(bool)"),self.chkAllMotors)
        QtCore.QObject.connect(self.ui.btnStopAll,QtCore.SIGNAL("toggled(bool)"),self.stopAll)

        self.ui.btnRec.setEnabled(self.ui.chkAllMotors.isChecked())
        self.totalFrames=0
        self.currentFrame=0
        self.btnInsert()

        self.ui.chkAllMotors.setEnabled(False)
        self.ui.chkR1.setEnabled(False)
        self.ui.chkR2.setEnabled(False)
        self.ui.chkR3.setEnabled(False)
        self.ui.chkR4.setEnabled(False)
        self.ui.chkR5.setEnabled(False)
        self.ui.chkR6.setEnabled(False)
        self.ui.chkL7.setEnabled(False)
        self.ui.chkL8.setEnabled(False)
        self.ui.chkL9.setEnabled(False)
        self.ui.chkL10.setEnabled(False)
        self.ui.chkL11.setEnabled(False)
        self.ui.chkL12.setEnabled(False)
        self.ui.chkRA.setEnabled(False)
        self.ui.chkLA.setEnabled(False)

        QtCore.QObject.connect(self.ui.btnRec,QtCore.SIGNAL("clicked()"),self.btnRec)
        QtCore.QObject.connect(self.ui.btnDelCurrent,QtCore.SIGNAL("clicked()"),self.btnDelCurrent)
        # QtCore.QObject.connect(self.ui.btnDel,QtCore.SIGNAL("clicked()"),self.btnDel)
        # QtCore.QObject.connect(self.ui.btnCopy,QtCore.SIGNAL("clicked()"),self.btnCopy)
        # QtCore.QObject.connect(self.ui.btnFirst,QtCore.SIGNAL("clicked()"),self.btnFirst)
        # QtCore.QObject.connect(self.ui.btnLast,QtCore.SIGNAL("clicked()"),self.btnLast)
        # QtCore.QObject.connect(self.ui.btnPrevious,QtCore.SIGNAL("clicked()"),self.btnPrevious)
        # QtCore.QObject.connect(self.ui.btnNext,QtCore.SIGNAL("clicked()"),self.btnNext)
        # QtCore.QObject.connect(self.ui.btnSave,QtCore.SIGNAL("clicked()"),self.btnSave)
        # QtCore.QObject.connect(self.ui.btnSaveAs,QtCore.SIGNAL("clicked()"),self.btnSaveAs)
        # QtCore.QObject.connect(self.ui.btnLoad,QtCore.SIGNAL("clicked()"),self.btnLoad)
        # QtCore.QObject.connect(self.ui.btnImport,QtCore.SIGNAL("clicked()"),self.btnImport)
        # QtCore.QObject.connect(self.ui.btnInsert,QtCore.SIGNAL("clicked()"),self.btnInsert)
        # QtCore.QObject.connect(self.ui.btnSetFrame,QtCore.SIGNAL("clicked()"),self.btnSetFrame)
        # QtCore.QObject.connect(self.ui.btnPlay,QtCore.SIGNAL("clicked()"),self.btnPlay)

        rospy.init_node("zeno_walk_tool")

        self.joint_positions = list(repeat(0.0, 12))
        self.joint_velocities = list(repeat(0.0, 12))
        # self.motorTorqueStates=list(repeat(False,12))
        for name in self.names:
            controller = name + '_controller/state'
            rospy.loginfo(controller)
            rospy.Subscriber(controller, DynamixelJointState, self.update_dynamixel_joint_state)

        self.setAllTorque(False)

    def update_dynamixel_joint_state(self, msg):
        joint_index = msg.motor_ids[0] - 1
        self.joint_positions[joint_index] = msg.current_pos
        # self.joint_velocities[joint_index] = msg.velocity
        # print(str(joint_index)+":"+str(self.joint_positions[joint_index])+"\n")

    def setFrameBtnState(self):
        if (self.currentFrame>=self.totalFrames):
            self.currentFrame=self.totalFrames
            self.ui.btnNext.setEnabled(False)
        else:
            self.ui.btnNext.setEnabled(True)

        if (self.currentFrame<=1):
            self.currentFrame=1
            self.ui.btnPrevious.setEnabled(False)
        else:
            self.ui.btnPrevious.setEnabled(True)

        self.ui.lblCurrFrame=str(self.currentFrame)
        self.ui.lblFrameCount=str(self.totalFrames)

    def btnRec(self):
        self.TimeDelayFromPrevious[self.currentFrame]=self.ui.spinDelay.value()
        for name in self.names:
            self.TrajectoryInfo[name][self.currentFrame]=self.joint_positions[self.nameAndIdIndex[name]-1]

    def btnDelCurrent(self):
        if self.totalFrames>1:
            self.TimeDelayFromPrevious.remove(self.currentFrame-1)#assuming frames start from 1 to n
            for name in self.names:
                self.TrajectoryInfo[name].remove(self.currentFrame-1)
            self.totalFrames=self.totalFrames-1
            self.setFrameBtnState()

    def btnDel(self):
        fcount=self.ui.spinDelTo.value()-self.ui.spinDelFrom.value()+1
        if fcount<1:
            return
        if self.totalFrames>fcount and self.ui.spinDelFrom.value()>=1 and self.ui.spinDelTo.value()<=self.totalFrames:
            for n in range(self.ui.spinDelTo.value(),self.ui.spinDelTo.value(),1):
                self.TimeDelayFromPrevious.remove(n-1)#assuming frames start from 1 to n
                for name in self.names:
                    self.TrajectoryInfo[name].remove(n-1)
            self.totalFrames=self.totalFrames-fcount
            self.setFrameBtnState()


    def btnCopy(self):
        if self.ui.spinCopyStart>=self.ui.spinCopyStop:
            return
        if self.ui.spinCopyTo.value()<1 or self.ui.spinCopyTo.value()>self.totalFrames:
            return
        if self.ui.spinCopyStart.value()>=1 and self.ui.spinCopyStop.value()<=self.totalFrames:
            #make a separate buffer
            tempDelayArr=[]
            tempTraj={}
            for name in self.names:
                tempTraj[name]=[]#pos

            for n in range(self.ui.spinCopyStart.value(),self.ui.spinCopyStop.value(),1):
                tempDelayArr.append(self.TimeDelayFromPrevious[n-1])
                for name in self.names:
                    tempTraj[name][n-self.ui.spinCopyStart.value()]=self.TrajectoryInfo[name][n]

            for n in range(0,self.ui.spinCopyStop.value()-self.ui.spinCopyStart.value(),1):
                newLoc=self.ui.spinCopyTo+n
                if newLoc>self.totalFrames:
                    self.TimeDelayFromPrevious.append(tempDelayArr[n])
                    for name in self.names:
                        self.TrajectoryInfo[name].append(tempTraj[n])
                else:
                    self.TimeDelayFromPrevious[newLoc]=tempDelayArr[n]
                    for name in self.names:
                        self.TrajectoryInfo[name][newLoc]=tempTraj[name][n]


    # def btnPlay(self):
    #
    # def btnSave(self):
    #
    # def btnSaveAs(self):
    #
    # def btnLoad(self):
    #
    # def btnImport(self):
    #
    def btnInsert(self):
        #insert a blank record
        if self.currentFrame==self.totalFrames:
            if self.totalFrames>0:
                self.TimeDelayFromPrevious.append(self.TimeDelayFromPrevious[self.currentFrame-1])
            else:
                self.TimeDelayFromPrevious.append(40)
            for name in self.names:
                if self.totalFrames>0:
                    self.TrajectoryInfo[name].append(self.TrajectoryInfo[name][self.currentFrame-1])
                else:
                    self.TrajectoryInfo[name].append(0.0)
        else:
            self.TimeDelayFromPrevious.insert(self.currentFrame,40)
            for name in self.names:
                self.TrajectoryInfo[name].insert(self.currentFrame,0.0)

        self.totalFrames=self.totalFrames+1
        self.currentFrame=self.currentFrame+1
        self.setFrameBtnState()
        self.ui.spinDelay.setValue(self.TimeDelayFromPrevious[self.currentFrame-1])


    def btnFirst(self):
        self.currentFrame=1
        self.setFrameBtnState()

    def btnLast(self):
        self.currentFrame=self.totalFrames
        self.setFrameBtnState()

    def btnPrevious(self):
        self.currentFrame=self.currentFrame-1
        self.setFrameBtnState()

    def btnNext(self):
        self.currentFrame=self.currentFrame+1
        self.setFrameBtnState()

    def btnSetFrame(self):
        self.currentFrame=self.ui.spinSetFrame.value()
        self.setFrameBtnState()

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

    def stopAll(self,stt):
        istt=not stt
        if stt:
            self.ui.chkAllMotors.setChecked(istt)
            self.ui.chkAllMotors.setChecked(istt)
            self.ui.chkR1.setChecked(istt)
            self.ui.chkR2.setChecked(istt)
            self.ui.chkR3.setChecked(istt)
            self.ui.chkR4.setChecked(istt)
            self.ui.chkR5.setChecked(istt)
            self.ui.chkR6.setChecked(istt)
            self.ui.chkL7.setChecked(istt)
            self.ui.chkL8.setChecked(istt)
            self.ui.chkL9.setChecked(istt)
            self.ui.chkL10.setChecked(istt)
            self.ui.chkL11.setChecked(istt)
            self.ui.chkL12.setChecked(istt)
            self.ui.chkRA.setChecked(istt)
            self.ui.chkLA.setChecked(istt)
        self.ui.chkAllMotors.setEnabled(istt)
        self.ui.chkR1.setEnabled(istt)
        self.ui.chkR2.setEnabled(istt)
        self.ui.chkR3.setEnabled(istt)
        self.ui.chkR4.setEnabled(istt)
        self.ui.chkR5.setEnabled(istt)
        self.ui.chkR6.setEnabled(istt)
        self.ui.chkL7.setEnabled(istt)
        self.ui.chkL8.setEnabled(istt)
        self.ui.chkL9.setEnabled(istt)
        self.ui.chkL10.setEnabled(istt)
        self.ui.chkL11.setEnabled(istt)
        self.ui.chkL12.setEnabled(istt)
        self.ui.chkRA.setEnabled(istt)
        self.ui.chkLA.setEnabled(istt)

    def chkR1(self,stt):
        print("chkR1="+str(stt))
        self.setMotorTorque(self.names[0],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkR2(self,stt):
        print("chkR2="+str(stt))
        self.setMotorTorque(self.names[1],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkR3(self,stt):
        print("chkR3="+str(stt))
        self.setMotorTorque(self.names[2],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkR4(self,stt):
        print("chkR4="+str(stt))
        self.setMotorTorque(self.names[3],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkR5(self,stt):
        print("chkR5="+str(stt))
        self.setMotorTorque(self.names[4],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkR6(self,stt):
        print("chkR6="+str(stt))
        self.setMotorTorque(self.names[5],stt)
        if not stt:
            self.ui.chkRA.setChecked(stt)

    def chkL7(self,stt):
        print("chkL7="+str(stt))
        self.setMotorTorque(self.names[6],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkL8(self,stt):
        print("chkL8="+str(stt))
        self.setMotorTorque(self.names[7],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkL9(self,stt):
        print("chkL9="+str(stt))
        self.setMotorTorque(self.names[8],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkL10(self,stt):
        print("chkL10="+str(stt))
        self.setMotorTorque(self.names[9],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkL11(self,stt):
        print("chkL11="+str(stt))
        self.setMotorTorque(self.names[10],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkL12(self,stt):
        print("chkL12="+str(stt))
        self.setMotorTorque(self.names[11],stt)
        if not stt:
            self.ui.chkLA.setChecked(stt)

    def chkLA(self,stt):
        print("chkLA="+str(stt))
        if stt:
            self.ui.chkL7.setChecked(stt)
            self.ui.chkL8.setChecked(stt)
            self.ui.chkL9.setChecked(stt)
            self.ui.chkL10.setChecked(stt)
            self.ui.chkL11.setChecked(stt)
            self.ui.chkL12.setChecked(stt)
        else:
            self.ui.chkAllMotors.setChecked(stt)

    def chkRA(self,stt):
        print("chkRA="+str(stt))
        if stt:
            self.ui.chkR1.setChecked(stt)
            self.ui.chkR2.setChecked(stt)
            self.ui.chkR3.setChecked(stt)
            self.ui.chkR4.setChecked(stt)
            self.ui.chkR5.setChecked(stt)
            self.ui.chkR6.setChecked(stt)
        else:
            self.ui.chkAllMotors.setChecked(stt)

    def chkAllMotors(self,stt):
        print("chkAll="+str(stt))
        if stt:
            self.ui.chkLA.setChecked(stt)
            self.ui.chkRA.setChecked(stt)
        self.ui.btnRec.setEnabled(stt)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = StartQT4()
    myapp.show()
    #app.processEvents()
    sys.exit(app.exec_())