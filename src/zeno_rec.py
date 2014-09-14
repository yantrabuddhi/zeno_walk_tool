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
    names = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll']
    motorNamesAndCat={'r_hip_yaw':'r', 'r_hip_roll':'r', 'r_hip_pitch':'r', 'r_knee_pitch':'r', 'r_ankle_pitch':'r', 'r_ankle_roll':'r',
'l_hip_yaw':'l', 'l_hip_roll':'l', 'l_hip_pitch':'l', 'l_knee_pitch':'l', 'l_ankle_pitch':'l', 'l_ankle_roll':'l'}#as in yaml array of tuple name,left or right
    motorPositionTopics=[]#List
    motorTorqueServices=[]#List
    motorTorqueStates=[]#bool List
    motorTrajectoryTopics=[]#dictionary category:TrajectoryTopic

    totalFrames=1
    currentFrame=1

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
        QtCore.QObject.connect(self.ui.btnDel,QtCore.SIGNAL("clicked()"),self.btnDel)
        QtCore.QObject.connect(self.ui.btnCopy,QtCore.SIGNAL("clicked()"),self.btnCopy)
        QtCore.QObject.connect(self.ui.btnFirst,QtCore.SIGNAL("clicked()"),self.btnFirst)
        QtCore.QObject.connect(self.ui.btnLast,QtCore.SIGNAL("clicked()"),self.btnLast)
        QtCore.QObject.connect(self.ui.btnSave,QtCore.SIGNAL("clicked()"),self.btnSave)
        QtCore.QObject.connect(self.ui.btnSaveAs,QtCore.SIGNAL("clicked()"),self.btnSaveAs)
        QtCore.QObject.connect(self.ui.btnLoad,QtCore.SIGNAL("clicked()"),self.btnLoad)
        QtCore.QObject.connect(self.ui.btnImport,QtCore.SIGNAL("clicked()"),self.btnImport)
        QtCore.QObject.connect(self.ui.btnInsert,QtCore.SIGNAL("clicked()"),self.btnInsert)
        QtCore.QObject.connect(self.ui.btnSetFrame,QtCore.SIGNAL("clicked()"),self.btnSetFrame)
        QtCore.QObject.connect(self.ui.btnPlay,QtCore.SIGNAL("clicked()"),self.btnPlay)

        rospy.init_node("zeno_walk_tool")

        self.joint_positions = list(repeat(0.0, 12))
        self.joint_velocities = list(repeat(0.0, 12))
        for name in self.names:
            controller = name + '_controller/state'
            rospy.loginfo(controller)
            rospy.Subscriber(controller, DynamixelJointState, self.update_dynamixel_joint_state)

        self.setAllTorque(False)

    def update_dynamixel_joint_state(self, msg):
        joint_index = msg.motor_ids[0] - 1
        self.joint_positions[joint_index] = msg.current_pos
        self.joint_velocities[joint_index] = msg.velocity
        #print to check updated joint pos?

    def btnRec(self):

    def btnDelCurrent(self):

    def btnDel(self):

    def btnCopy(self):

    def btnFirst(self):

    def btnLast(self):

    def btnSave(self):

    def btnSaveAs(self):

    def btnLoad(self):

    def btnImport(self):

    def btnInsert(self):

    def btnSetFrame(self):

    def btnPlay(self):

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