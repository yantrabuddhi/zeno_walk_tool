# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'UI_zeno_rec.ui'
#
# Created: Tue Sep 16 14:22:45 2014
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_zeno_rec(object):
    def setupUi(self, zeno_rec):
        zeno_rec.setObjectName(_fromUtf8("zeno_rec"))
        zeno_rec.resize(1008, 650)
        self.frameLeft = QtGui.QFrame(zeno_rec)
        self.frameLeft.setGeometry(QtCore.QRect(10, 20, 291, 341))
        self.frameLeft.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frameLeft.setFrameShadow(QtGui.QFrame.Raised)
        self.frameLeft.setObjectName(_fromUtf8("frameLeft"))
        self.grpLeft = QtGui.QGroupBox(self.frameLeft)
        self.grpLeft.setGeometry(QtCore.QRect(10, 10, 261, 321))
        self.grpLeft.setObjectName(_fromUtf8("grpLeft"))
        self.lblL7 = QtGui.QLabel(self.grpLeft)
        self.lblL7.setGeometry(QtCore.QRect(10, 40, 171, 21))
        self.lblL7.setObjectName(_fromUtf8("lblL7"))
        self.chkL7 = QtGui.QCheckBox(self.grpLeft)
        self.chkL7.setGeometry(QtCore.QRect(200, 40, 51, 22))
        self.chkL7.setObjectName(_fromUtf8("chkL7"))
        self.chkL8 = QtGui.QCheckBox(self.grpLeft)
        self.chkL8.setGeometry(QtCore.QRect(200, 80, 51, 22))
        self.chkL8.setObjectName(_fromUtf8("chkL8"))
        self.lblL8 = QtGui.QLabel(self.grpLeft)
        self.lblL8.setGeometry(QtCore.QRect(10, 80, 171, 21))
        self.lblL8.setObjectName(_fromUtf8("lblL8"))
        self.chkL9 = QtGui.QCheckBox(self.grpLeft)
        self.chkL9.setGeometry(QtCore.QRect(200, 120, 51, 22))
        self.chkL9.setObjectName(_fromUtf8("chkL9"))
        self.lblL9 = QtGui.QLabel(self.grpLeft)
        self.lblL9.setGeometry(QtCore.QRect(10, 120, 171, 21))
        self.lblL9.setObjectName(_fromUtf8("lblL9"))
        self.chkL10 = QtGui.QCheckBox(self.grpLeft)
        self.chkL10.setGeometry(QtCore.QRect(200, 160, 51, 22))
        self.chkL10.setObjectName(_fromUtf8("chkL10"))
        self.lblL10 = QtGui.QLabel(self.grpLeft)
        self.lblL10.setGeometry(QtCore.QRect(10, 160, 171, 21))
        self.lblL10.setObjectName(_fromUtf8("lblL10"))
        self.chkL11 = QtGui.QCheckBox(self.grpLeft)
        self.chkL11.setGeometry(QtCore.QRect(200, 200, 51, 22))
        self.chkL11.setObjectName(_fromUtf8("chkL11"))
        self.lblL11 = QtGui.QLabel(self.grpLeft)
        self.lblL11.setGeometry(QtCore.QRect(10, 200, 171, 21))
        self.lblL11.setObjectName(_fromUtf8("lblL11"))
        self.chkL12 = QtGui.QCheckBox(self.grpLeft)
        self.chkL12.setGeometry(QtCore.QRect(200, 240, 51, 22))
        self.chkL12.setObjectName(_fromUtf8("chkL12"))
        self.lblL12 = QtGui.QLabel(self.grpLeft)
        self.lblL12.setGeometry(QtCore.QRect(10, 240, 171, 21))
        self.lblL12.setObjectName(_fromUtf8("lblL12"))
        self.lblLALL = QtGui.QLabel(self.grpLeft)
        self.lblLALL.setGeometry(QtCore.QRect(10, 280, 171, 21))
        self.lblLALL.setObjectName(_fromUtf8("lblLALL"))
        self.chkLA = QtGui.QCheckBox(self.grpLeft)
        self.chkLA.setGeometry(QtCore.QRect(200, 280, 51, 22))
        self.chkLA.setObjectName(_fromUtf8("chkLA"))
        self.lblLT = QtGui.QLabel(self.grpLeft)
        self.lblLT.setGeometry(QtCore.QRect(200, 20, 51, 17))
        self.lblLT.setObjectName(_fromUtf8("lblLT"))
        self.frameRight = QtGui.QFrame(zeno_rec)
        self.frameRight.setGeometry(QtCore.QRect(320, 20, 291, 341))
        self.frameRight.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frameRight.setFrameShadow(QtGui.QFrame.Raised)
        self.frameRight.setObjectName(_fromUtf8("frameRight"))
        self.grpRight = QtGui.QGroupBox(self.frameRight)
        self.grpRight.setGeometry(QtCore.QRect(10, 10, 261, 321))
        self.grpRight.setObjectName(_fromUtf8("grpRight"))
        self.lblR1 = QtGui.QLabel(self.grpRight)
        self.lblR1.setGeometry(QtCore.QRect(10, 40, 171, 21))
        self.lblR1.setObjectName(_fromUtf8("lblR1"))
        self.chkR1 = QtGui.QCheckBox(self.grpRight)
        self.chkR1.setGeometry(QtCore.QRect(200, 40, 51, 22))
        self.chkR1.setObjectName(_fromUtf8("chkR1"))
        self.chkR2 = QtGui.QCheckBox(self.grpRight)
        self.chkR2.setGeometry(QtCore.QRect(200, 80, 51, 22))
        self.chkR2.setObjectName(_fromUtf8("chkR2"))
        self.lblR2 = QtGui.QLabel(self.grpRight)
        self.lblR2.setGeometry(QtCore.QRect(10, 80, 171, 21))
        self.lblR2.setObjectName(_fromUtf8("lblR2"))
        self.chkR3 = QtGui.QCheckBox(self.grpRight)
        self.chkR3.setGeometry(QtCore.QRect(200, 120, 51, 22))
        self.chkR3.setObjectName(_fromUtf8("chkR3"))
        self.lblR3 = QtGui.QLabel(self.grpRight)
        self.lblR3.setGeometry(QtCore.QRect(10, 120, 171, 21))
        self.lblR3.setObjectName(_fromUtf8("lblR3"))
        self.chkR4 = QtGui.QCheckBox(self.grpRight)
        self.chkR4.setGeometry(QtCore.QRect(200, 160, 51, 22))
        self.chkR4.setObjectName(_fromUtf8("chkR4"))
        self.lblR4 = QtGui.QLabel(self.grpRight)
        self.lblR4.setGeometry(QtCore.QRect(10, 160, 171, 21))
        self.lblR4.setObjectName(_fromUtf8("lblR4"))
        self.chkR5 = QtGui.QCheckBox(self.grpRight)
        self.chkR5.setGeometry(QtCore.QRect(200, 200, 51, 22))
        self.chkR5.setObjectName(_fromUtf8("chkR5"))
        self.lblR5 = QtGui.QLabel(self.grpRight)
        self.lblR5.setGeometry(QtCore.QRect(10, 200, 171, 21))
        self.lblR5.setObjectName(_fromUtf8("lblR5"))
        self.chkR6 = QtGui.QCheckBox(self.grpRight)
        self.chkR6.setGeometry(QtCore.QRect(200, 240, 51, 22))
        self.chkR6.setObjectName(_fromUtf8("chkR6"))
        self.lblR6 = QtGui.QLabel(self.grpRight)
        self.lblR6.setGeometry(QtCore.QRect(10, 240, 171, 21))
        self.lblR6.setObjectName(_fromUtf8("lblR6"))
        self.lblRALL = QtGui.QLabel(self.grpRight)
        self.lblRALL.setGeometry(QtCore.QRect(10, 280, 171, 21))
        self.lblRALL.setObjectName(_fromUtf8("lblRALL"))
        self.chkRA = QtGui.QCheckBox(self.grpRight)
        self.chkRA.setGeometry(QtCore.QRect(200, 280, 51, 22))
        self.chkRA.setObjectName(_fromUtf8("chkRA"))
        self.lblLT_2 = QtGui.QLabel(self.grpRight)
        self.lblLT_2.setGeometry(QtCore.QRect(200, 20, 51, 17))
        self.lblLT_2.setObjectName(_fromUtf8("lblLT_2"))
        self.btnSave = QtGui.QPushButton(zeno_rec)
        self.btnSave.setGeometry(QtCore.QRect(10, 610, 71, 27))
        self.btnSave.setObjectName(_fromUtf8("btnSave"))
        self.btnSaveAs = QtGui.QPushButton(zeno_rec)
        self.btnSaveAs.setGeometry(QtCore.QRect(100, 610, 71, 27))
        self.btnSaveAs.setObjectName(_fromUtf8("btnSaveAs"))
        self.btnLoad = QtGui.QPushButton(zeno_rec)
        self.btnLoad.setGeometry(QtCore.QRect(190, 610, 71, 27))
        self.btnLoad.setObjectName(_fromUtf8("btnLoad"))
        self.chkAllMotors = QtGui.QCheckBox(zeno_rec)
        self.chkAllMotors.setGeometry(QtCore.QRect(220, 380, 171, 22))
        self.chkAllMotors.setObjectName(_fromUtf8("chkAllMotors"))
        self.label = QtGui.QLabel(zeno_rec)
        self.label.setGeometry(QtCore.QRect(10, 400, 141, 41))
        self.label.setObjectName(_fromUtf8("label"))
        self.spinDelay = QtGui.QSpinBox(zeno_rec)
        self.spinDelay.setGeometry(QtCore.QRect(170, 410, 71, 27))
        self.spinDelay.setMinimum(40)
        self.spinDelay.setMaximum(2000)
        self.spinDelay.setSingleStep(100)
        self.spinDelay.setObjectName(_fromUtf8("spinDelay"))
        self.spinSetFrame = QtGui.QSpinBox(zeno_rec)
        self.spinSetFrame.setGeometry(QtCore.QRect(160, 490, 91, 27))
        self.spinSetFrame.setMaximum(9999)
        self.spinSetFrame.setObjectName(_fromUtf8("spinSetFrame"))
        self.btnInsert = QtGui.QPushButton(zeno_rec)
        self.btnInsert.setGeometry(QtCore.QRect(290, 450, 141, 27))
        self.btnInsert.setObjectName(_fromUtf8("btnInsert"))
        self.btnDelCurrent = QtGui.QPushButton(zeno_rec)
        self.btnDelCurrent.setGeometry(QtCore.QRect(450, 450, 161, 27))
        self.btnDelCurrent.setObjectName(_fromUtf8("btnDelCurrent"))
        self.btnFirst = QtGui.QPushButton(zeno_rec)
        self.btnFirst.setGeometry(QtCore.QRect(310, 530, 121, 27))
        self.btnFirst.setObjectName(_fromUtf8("btnFirst"))
        self.btnLast = QtGui.QPushButton(zeno_rec)
        self.btnLast.setGeometry(QtCore.QRect(450, 530, 121, 27))
        self.btnLast.setObjectName(_fromUtf8("btnLast"))
        self.btnRec = QtGui.QPushButton(zeno_rec)
        self.btnRec.setGeometry(QtCore.QRect(440, 380, 101, 27))
        self.btnRec.setObjectName(_fromUtf8("btnRec"))
        self.frame = QtGui.QFrame(zeno_rec)
        self.frame.setGeometry(QtCore.QRect(620, 20, 381, 171))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.groupBox = QtGui.QGroupBox(self.frame)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 351, 151))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.label_3 = QtGui.QLabel(self.groupBox)
        self.label_3.setGeometry(QtCore.QRect(20, 30, 91, 17))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.groupBox)
        self.label_4.setGeometry(QtCore.QRect(20, 90, 66, 17))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.label_5 = QtGui.QLabel(self.groupBox)
        self.label_5.setGeometry(QtCore.QRect(20, 60, 81, 17))
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.spinCopyStart = QtGui.QSpinBox(self.groupBox)
        self.spinCopyStart.setGeometry(QtCore.QRect(130, 20, 81, 27))
        self.spinCopyStart.setObjectName(_fromUtf8("spinCopyStart"))
        self.spinCopyStop = QtGui.QSpinBox(self.groupBox)
        self.spinCopyStop.setGeometry(QtCore.QRect(130, 50, 81, 27))
        self.spinCopyStop.setObjectName(_fromUtf8("spinCopyStop"))
        self.spinCopyTo = QtGui.QSpinBox(self.groupBox)
        self.spinCopyTo.setGeometry(QtCore.QRect(130, 80, 81, 27))
        self.spinCopyTo.setObjectName(_fromUtf8("spinCopyTo"))
        self.btnCopy = QtGui.QPushButton(self.groupBox)
        self.btnCopy.setGeometry(QtCore.QRect(20, 120, 98, 27))
        self.btnCopy.setObjectName(_fromUtf8("btnCopy"))
        self.btnImport = QtGui.QPushButton(self.groupBox)
        self.btnImport.setGeometry(QtCore.QRect(250, 20, 91, 27))
        self.btnImport.setObjectName(_fromUtf8("btnImport"))
        self.label_6 = QtGui.QLabel(zeno_rec)
        self.label_6.setGeometry(QtCore.QRect(10, 380, 91, 17))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.frame_2 = QtGui.QFrame(zeno_rec)
        self.frame_2.setGeometry(QtCore.QRect(620, 200, 381, 161))
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName(_fromUtf8("frame_2"))
        self.groupBox_2 = QtGui.QGroupBox(self.frame_2)
        self.groupBox_2.setGeometry(QtCore.QRect(10, 10, 361, 141))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.label_7 = QtGui.QLabel(self.groupBox_2)
        self.label_7.setGeometry(QtCore.QRect(20, 40, 91, 17))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.label_8 = QtGui.QLabel(self.groupBox_2)
        self.label_8.setGeometry(QtCore.QRect(20, 120, 66, 17))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.spinDelFrom = QtGui.QSpinBox(self.groupBox_2)
        self.spinDelFrom.setGeometry(QtCore.QRect(130, 30, 81, 27))
        self.spinDelFrom.setObjectName(_fromUtf8("spinDelFrom"))
        self.spinDelTo = QtGui.QSpinBox(self.groupBox_2)
        self.spinDelTo.setGeometry(QtCore.QRect(130, 110, 81, 27))
        self.spinDelTo.setObjectName(_fromUtf8("spinDelTo"))
        self.btnDel = QtGui.QPushButton(self.groupBox_2)
        self.btnDel.setGeometry(QtCore.QRect(240, 110, 98, 27))
        self.btnDel.setObjectName(_fromUtf8("btnDel"))
        self.frame_3 = QtGui.QFrame(zeno_rec)
        self.frame_3.setGeometry(QtCore.QRect(620, 380, 381, 261))
        self.frame_3.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_3.setObjectName(_fromUtf8("frame_3"))
        self.groupBox_3 = QtGui.QGroupBox(self.frame_3)
        self.groupBox_3.setGeometry(QtCore.QRect(10, 10, 361, 241))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.chkStopWatch = QtGui.QCheckBox(self.groupBox_3)
        self.chkStopWatch.setGeometry(QtCore.QRect(20, 50, 141, 22))
        self.chkStopWatch.setObjectName(_fromUtf8("chkStopWatch"))
        self.lblWatch = QtGui.QLabel(self.groupBox_3)
        self.lblWatch.setGeometry(QtCore.QRect(170, 50, 61, 17))
        self.lblWatch.setObjectName(_fromUtf8("lblWatch"))
        self.btnResetWatch = QtGui.QPushButton(self.groupBox_3)
        self.btnResetWatch.setGeometry(QtCore.QRect(250, 40, 71, 27))
        self.btnResetWatch.setObjectName(_fromUtf8("btnResetWatch"))
        self.btnPrintCurrent = QtGui.QPushButton(self.groupBox_3)
        self.btnPrintCurrent.setGeometry(QtCore.QRect(30, 190, 98, 27))
        self.btnPrintCurrent.setObjectName(_fromUtf8("btnPrintCurrent"))
        self.lblFrameCount = QtGui.QLabel(zeno_rec)
        self.lblFrameCount.setGeometry(QtCore.QRect(130, 380, 66, 17))
        self.lblFrameCount.setObjectName(_fromUtf8("lblFrameCount"))
        self.btnPlay = QtGui.QPushButton(zeno_rec)
        self.btnPlay.setGeometry(QtCore.QRect(50, 530, 61, 27))
        self.btnPlay.setObjectName(_fromUtf8("btnPlay"))
        self.spinPlayTo = QtGui.QSpinBox(zeno_rec)
        self.spinPlayTo.setGeometry(QtCore.QRect(160, 530, 91, 27))
        self.spinPlayTo.setObjectName(_fromUtf8("spinPlayTo"))
        self.label_10 = QtGui.QLabel(zeno_rec)
        self.label_10.setGeometry(QtCore.QRect(120, 540, 31, 17))
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.btnSetFrame = QtGui.QPushButton(zeno_rec)
        self.btnSetFrame.setGeometry(QtCore.QRect(10, 490, 98, 27))
        self.btnSetFrame.setObjectName(_fromUtf8("btnSetFrame"))
        self.label_2 = QtGui.QLabel(zeno_rec)
        self.label_2.setGeometry(QtCore.QRect(390, 420, 111, 17))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.lblCurrFrame = QtGui.QLabel(zeno_rec)
        self.lblCurrFrame.setGeometry(QtCore.QRect(510, 420, 66, 17))
        self.lblCurrFrame.setObjectName(_fromUtf8("lblCurrFrame"))
        self.btnPrevious = QtGui.QPushButton(zeno_rec)
        self.btnPrevious.setGeometry(QtCore.QRect(360, 490, 71, 27))
        self.btnPrevious.setObjectName(_fromUtf8("btnPrevious"))
        self.btnNext = QtGui.QPushButton(zeno_rec)
        self.btnNext.setGeometry(QtCore.QRect(450, 490, 71, 27))
        self.btnNext.setObjectName(_fromUtf8("btnNext"))
        self.line = QtGui.QFrame(zeno_rec)
        self.line.setGeometry(QtCore.QRect(260, 460, 20, 121))
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.label_9 = QtGui.QLabel(zeno_rec)
        self.label_9.setGeometry(QtCore.QRect(10, 580, 31, 17))
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.lblFile = QtGui.QLabel(zeno_rec)
        self.lblFile.setGeometry(QtCore.QRect(50, 580, 551, 17))
        self.lblFile.setObjectName(_fromUtf8("lblFile"))
        self.btnStopAll = QtGui.QPushButton(zeno_rec)
        self.btnStopAll.setGeometry(QtCore.QRect(470, 610, 141, 31))
        self.btnStopAll.setCheckable(True)
        self.btnStopAll.setChecked(True)
        self.btnStopAll.setObjectName(_fromUtf8("btnStopAll"))
        self.btnRecDelay = QtGui.QPushButton(zeno_rec)
        self.btnRecDelay.setGeometry(QtCore.QRect(260, 410, 98, 27))
        self.btnRecDelay.setObjectName(_fromUtf8("btnRecDelay"))

        self.retranslateUi(zeno_rec)
        QtCore.QMetaObject.connectSlotsByName(zeno_rec)

    def retranslateUi(self, zeno_rec):
        zeno_rec.setWindowTitle(_translate("zeno_rec", "Zeno Leg Animation", None))
        self.grpLeft.setTitle(_translate("zeno_rec", "Left Leg", None))
        self.lblL7.setText(_translate("zeno_rec", "Hip Yaw", None))
        self.chkL7.setText(_translate("zeno_rec", "7", None))
        self.chkL8.setText(_translate("zeno_rec", "8", None))
        self.lblL8.setText(_translate("zeno_rec", "Hip Roll", None))
        self.chkL9.setText(_translate("zeno_rec", "9", None))
        self.lblL9.setText(_translate("zeno_rec", "Hip Pitch", None))
        self.chkL10.setText(_translate("zeno_rec", "10", None))
        self.lblL10.setText(_translate("zeno_rec", "knee pitch", None))
        self.chkL11.setText(_translate("zeno_rec", "11", None))
        self.lblL11.setText(_translate("zeno_rec", "Ankle pitch", None))
        self.chkL12.setText(_translate("zeno_rec", "12", None))
        self.lblL12.setText(_translate("zeno_rec", "Ankle Roll", None))
        self.lblLALL.setText(_translate("zeno_rec", "Left", None))
        self.chkLA.setText(_translate("zeno_rec", "All", None))
        self.lblLT.setText(_translate("zeno_rec", "Torque", None))
        self.grpRight.setTitle(_translate("zeno_rec", "Right Leg", None))
        self.lblR1.setText(_translate("zeno_rec", "Hip Yaw", None))
        self.chkR1.setText(_translate("zeno_rec", "1", None))
        self.chkR2.setText(_translate("zeno_rec", "2", None))
        self.lblR2.setText(_translate("zeno_rec", "Hip Roll", None))
        self.chkR3.setText(_translate("zeno_rec", "3", None))
        self.lblR3.setText(_translate("zeno_rec", "Hip Pitch", None))
        self.chkR4.setText(_translate("zeno_rec", "4", None))
        self.lblR4.setText(_translate("zeno_rec", "knee pitch", None))
        self.chkR5.setText(_translate("zeno_rec", "5", None))
        self.lblR5.setText(_translate("zeno_rec", "Ankle pitch", None))
        self.chkR6.setText(_translate("zeno_rec", "6", None))
        self.lblR6.setText(_translate("zeno_rec", "Ankle Roll", None))
        self.lblRALL.setText(_translate("zeno_rec", "Right", None))
        self.chkRA.setText(_translate("zeno_rec", "All", None))
        self.lblLT_2.setText(_translate("zeno_rec", "Torque", None))
        self.btnSave.setText(_translate("zeno_rec", "Save", None))
        self.btnSaveAs.setText(_translate("zeno_rec", "Save As", None))
        self.btnLoad.setText(_translate("zeno_rec", "Load", None))
        self.chkAllMotors.setText(_translate("zeno_rec", "Torque for All Motors", None))
        self.label.setText(_translate("zeno_rec", "Elapse from previous\n"
"keyframe(m.sec):", None))
        self.btnInsert.setText(_translate("zeno_rec", "Insert Frame After", None))
        self.btnDelCurrent.setText(_translate("zeno_rec", "Delete Current Frame", None))
        self.btnFirst.setText(_translate("zeno_rec", "Goto First Frame", None))
        self.btnLast.setText(_translate("zeno_rec", "Goto Last Frame", None))
        self.btnRec.setText(_translate("zeno_rec", "Record Frame", None))
        self.groupBox.setTitle(_translate("zeno_rec", "Copy", None))
        self.label_3.setText(_translate("zeno_rec", "Start Frame", None))
        self.label_4.setText(_translate("zeno_rec", "To Frame", None))
        self.label_5.setText(_translate("zeno_rec", "Stop Frame", None))
        self.btnCopy.setText(_translate("zeno_rec", "Copy", None))
        self.btnImport.setText(_translate("zeno_rec", "Import", None))
        self.label_6.setText(_translate("zeno_rec", "Total Frames:", None))
        self.groupBox_2.setTitle(_translate("zeno_rec", "Delete", None))
        self.label_7.setText(_translate("zeno_rec", "From Frame", None))
        self.label_8.setText(_translate("zeno_rec", "To Frame", None))
        self.btnDel.setText(_translate("zeno_rec", "Delete", None))
        self.groupBox_3.setTitle(_translate("zeno_rec", "Tools", None))
        self.chkStopWatch.setText(_translate("zeno_rec", "Start Stopwatch", None))
        self.lblWatch.setText(_translate("zeno_rec", "00000", None))
        self.btnResetWatch.setText(_translate("zeno_rec", "Reset", None))
        self.btnPrintCurrent.setText(_translate("zeno_rec", "Print Current", None))
        self.lblFrameCount.setText(_translate("zeno_rec", "1", None))
        self.btnPlay.setText(_translate("zeno_rec", "Play", None))
        self.label_10.setText(_translate("zeno_rec", "To", None))
        self.btnSetFrame.setText(_translate("zeno_rec", "Set Frame", None))
        self.label_2.setText(_translate("zeno_rec", "Current Frame:", None))
        self.lblCurrFrame.setText(_translate("zeno_rec", "1", None))
        self.btnPrevious.setText(_translate("zeno_rec", "Previous", None))
        self.btnNext.setText(_translate("zeno_rec", "Next", None))
        self.label_9.setText(_translate("zeno_rec", "File:", None))
        self.lblFile.setText(_translate("zeno_rec", "default.dyn", None))
        self.btnStopAll.setText(_translate("zeno_rec", "Emergency Stop", None))
        self.btnRecDelay.setText(_translate("zeno_rec", "Rec Delay", None))

