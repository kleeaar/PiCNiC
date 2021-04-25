"""Copyright (c) 2021 L. Schimpf

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib import cm

from matplotlib.figure import Figure
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

from stl import mesh

import numpy as np
import time
import sys
from time import sleep

from includes.misc.ui import Ui_MainWindow
import includes.misc.utils as util

import includes.toolpath.lineScan as lineScan
import includes.toolpath.stlToContours as stlToContours
import includes.toolpath.svgToContours as svgToContours
from includes.communication.raspberryCommunication import raspberryCommunication
from includes.communication.threadedTasks import threadedTasks


import includes.misc.constants as constants
#pip install git+https://github.com/pyqtgraph/pyqtgraph required to fix retina display bug

sys.setrecursionlimit(1000000)

class myCNC():
    def __init__(self):
        self.currentPosition={'x':{'mm':0,'steps':0},'y':{'mm':0,'steps':0},'z':{'mm':0,'steps':0}}
        self.slowStep=50#steps
        self.fastStep=500#steps
        #self.stepLength=0.075#mm
        self.stepsPerMillimeter={'x':20,'y':20,'z':50} #without microstepping
        self.currentProgramStep=0
        self.programIsPaused=False
        self.stepBeforePause=0
        self.homePosition={'x':0,'y':0,'z':0}
        self.meshItem=None
        self.contourItem=None
        self.machineDimensions={'x':400,'y':400,'z':150}#mm
        self.stlScaling=1
        self.svgScaling=1
        self.stlFileName=None

    def setZero(self):
        self.currentPosition={'x':{'mm':0,'steps':0},'y':{'mm':0,'steps':0},'z':{'mm':0,'steps':0}}
        win.currentXpos.setText('{:.2f} mm'.format(0))
        win.xPosValue.setText('{:.2f} mm'.format(0))
        win.currentYpos.setText('{:.2f} mm'.format(0))
        win.yPosValue.setText('{:.2f} mm'.format(0))
        win.currentZpos.setText('{:.2f} mm'.format(0))
        win.zPosValue.setText('{:.2f} mm'.format(0))

    def setHomePosition(self):
        self.homePosition['x']=self.currentPosition['x']['mm']
        self.homePosition['y']=self.currentPosition['y']['mm']
        self.homePosition['z']=self.currentPosition['z']['mm']

    def translateToOrigin(self,meshItem=None):
        self.partMinZ=min(self.facets[...,2][...,2])
        self.partMaxZ=max(self.facets[...,2][...,2])#-self.partMinZ
        self.partMaxHeight=self.partMaxZ-self.partMinZ
        self.partMinX=min(self.facets[...,0][...,0])
        self.partMinY=min(self.facets[...,1][...,1])
        self.facets[...][...,0]-=self.partMinX
        self.facets[...][...,1]-=self.partMinY
        self.facets[...][...,2]-=self.partMaxZ

        if meshItem!=None:
            meshItem.translate(-self.partMinX,-self.partMinY,-self.partMaxZ)

    def generateStlToolpath(self):
        if win.processingStrategy.currentText()=='line':
            myLineScanObject=lineScan.lineScan(self.stlScaling*self.facets, 0.01,0.5, win.toolDiameterStl.value())
            #win.generateProgressStl.setText("Performing linescan")
            win.generateProgressStl.setText("Step 1 of 4: Analyzing shape")
            win.generateProgressStl.repaint()
            contour=myLineScanObject.multiThreadedLineScan(win.sliceDistance.value(),int(win.numberOfLayersStl.value()))
            win.generateProgressStl.setText("Step 2 of 4: Generating toolpath")
            win.generateProgressStl.repaint()
            self.toolpath=myLineScanObject.makeToolPath(np.asarray(contour),win.safetyDistanceStl.value())
            win.generateProgressStl.setText("Step 3 of 4: Flattening path")
            win.generateProgressStl.repaint()
            self.toolpath=myLineScanObject.flatten(self.toolpath)
            win.generateProgressStl.setText("Step 4 of 4: Simplifying path")
            win.generateProgressStl.repaint()
            self.toolpath=util.removeSameDirectionSteps(self.toolpath)
            #self.toolpath=util.shiftToolpath(self.toolpath,[-self.partMinX,-self.partMinY,-self.partMaxZ])
            win.generateProgressStl.setText("Done")
            win.generateProgressStl.repaint()
        elif win.processingStrategy.currentText()=='contour':
            inv=1
            if win.invertContour.isChecked():
                inv=-1
            if self.stlFileName is not None:
                contour=stlToContours.stlToContours(self.stlFileName)
                self.toolpath=contour.getContourToolpath(win.sliceDistance.value(),scalingFactor=self.stlScaling,offset=inv*win.toolDiameterStl.value()/2.,safetyDistance=win.safetyDistanceStl.value(), layers=int(win.numberOfLayersStl.value()))
            else:
                return
        elif win.processingStrategy.currentText()=='constant z':
            print(win.processingStrategy.currentText())
        win.plotToolpath(win.canvas,win.ax)

    def stlImport(self,fileName):
        self.stlFileName=fileName
        self.importedMesh = mesh.Mesh.from_file(fileName)
        self.facets=self.importedMesh.vectors
        #win.clearCanvasElements(win.plot3d)
        win.make3dPlot(win.canvas,win.ax)

    def generateSvgToolpath(self):
        if win.svgProcessingStrategy.currentText()=='contour':
            win.generateProgressSvg.setText('Analyzing contours')
            win.generateProgressSvg.repaint()
            QtWidgets.QApplication.processEvents()
            self.toolpath=self.contourClass.getContourToolpath(win.depth.value(),scalingFactor=self.svgScaling/constants.conversionFactorPPItoMM,offset=win.toolDiameter.value()/2.,safetyDistance=win.safetyDistance.value(), layers=int(win.numberOfLayers.value()))
        elif win.svgProcessingStrategy.currentText()=='constant z':
            win.generateProgressSvg.setText('Analyzing contours')
            win.generateProgressSvg.repaint()
            QtWidgets.QApplication.processEvents()
            self.toolpath=self.contourClass.makeOuterOffsets(win.depth.value(),win.offsetX.value(),win.offsetY.value(),scalingFactor=self.svgScaling/constants.conversionFactorPPItoMM,toolDimater=win.toolDiameter.value(),safetyDistance=win.safetyDistance.value(), layers=int(win.numberOfLayers.value()))

        win.generateProgressSvg.setText("Done")
        win.plotToolpath(win.canvas,win.ax)

    def svgImport(self,fileName):
        self.contourClass=svgToContours.svgToContours(fileName)
        self.importedContourForPlotting=np.asarray(self.contourClass.getContours())
        win.make2dPlot(win.canvas,win.ax,scaling=self.svgScaling/constants.conversionFactorPPItoMM)

class stepperControl():
    sendEndswitchValues=QtCore.pyqtSignal(object)
    def __init__(self, cncObject, winObject):
        self.cncObject=cncObject
        self.axisDirectionPin={'x':1, 'y':1, 'z':1}
        self.myRaspberryCommunication=raspberryCommunication()
        self.myRaspberryCommunication.changeMillingMotorStatus(0)

        self.cncObject.stepsPerMillimeter={'x':20*self.myRaspberryCommunication.microStepping['x'],'y':20*self.myRaspberryCommunication.microStepping['y'],'z':50*self.myRaspberryCommunication.microStepping['z']}
        self.threadedManualDrive=None
        self.threadedReferenceScan=None
        self.threadedProgram=None
        self.alreadyProcessedSteps=np.empty((0,3))
        self.isProgram=False
        self.millingMotorStatus=0
        self.win=winObject
        self.watchEndswitches()
        self.myRaspberryCommunication.turnOnOffFan(1, 100)

    def endSwitchesWatchdog(self):
        self.endswitchesStates=self.myRaspberryCommunication.getEndswitchesStates()
        buttons={ 'x':{'front':win.endswitchXFront,'rear':win.endswitchXRear}, 'y1':{'front':win.endswitchYFront1,'rear':win.endswitchYRear1}, 'y2':{'front':win.endswitchYFront2,'rear':win.endswitchYRear2},'z':{'front':win.endswitchZFront,'rear':win.endswitchZRear}}

        for axis in self.endswitchesStates:
            if axis=='allAxes':
                continue
            for switch in self.endswitchesStates[axis]:
                if self.endswitchesStates[axis][switch]==1:
                    buttons[axis][switch].setStyleSheet("QWidget { background: qradialgradient(cx: 0.5, cy: 0.5, fx: 0.5, fy: 0.5, radius: 1, stop: 0 #f75a00, stop: 1 #ea3e00); border-radius: 10px;border: 2px solid rgb(183, 186, 186);}")
                else:
                    buttons[axis][switch].setStyleSheet("QWidget { background: qradialgradient(cx: 0.5, cy: 0.5, fx: 0.5, fy: 0.5, radius: 1, stop: 0 #e1ff00, stop: 1 #7ae500); border-radius: 10px;border: 2px solid rgb(183, 186, 186);}")

        if self.endswitchesStates['allAxes']==1:
            win.runButton.setDisabled(True)


    def watchEndswitches(self):
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(False)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.endSwitchesWatchdog)
        self.timer.start()

    def watchToolPosition(self):
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(False)
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.win.makeToolMesh)
        self.timer.start()

    def setDirection(self, axis, direction):
        print('Set axis direction')

    def goHome(self):
        self.goToPosition(self.cncObject.homePosition['x'],self.cncObject.homePosition['y'],self.cncObject.homePosition['z'],manual=True)

    def goToPosition(self, x, y, z,manual=False):
        if manual:
            win.joggPosXSlow.setDisabled(True)
            win.joggPosXFast.setDisabled(True)
            win.joggNegXSlow.setDisabled(True)
            win.joggNegXFast.setDisabled(True)
            win.joggPosYSlow.setDisabled(True)
            win.joggPosYFast.setDisabled(True)
            win.joggNegYSlow.setDisabled(True)
            win.joggNegYFast.setDisabled(True)
            win.joggPosZSlow.setDisabled(True)
            win.joggPosZFast.setDisabled(True)
            win.joggNegZSlow.setDisabled(True)
            win.joggNegZFast.setDisabled(True)
            win.stopManualDrive.setEnabled(True)
            self.cncObject.programIsPaused=False
            win.goToPosition.setDisabled(True)
        deltaX=x-self.cncObject.currentPosition['x']['mm']
        deltaY=y-self.cncObject.currentPosition['y']['mm']
        deltaZ=z-self.cncObject.currentPosition['z']['mm']

        self.manualDriveThread=QtCore.QThread()
        self.threadedManualDrive=threadedTasks(self.cncObject.stepsPerMillimeter)
        self.threadedManualDrive.done.connect(self.manualDriveDone)
        self.threadedManualDrive.currentPosition=self.cncObject.currentPosition
        self.threadedManualDrive.targetPosition={'x':x,'y':y,'z':z}
        self.threadedManualDrive.stepsPerMillimeter=self.cncObject.stepsPerMillimeter
        self.threadedManualDrive.speed=win.speedManualDrive.value()
        self.threadedManualDrive.endswitchesStates=self.myRaspberryCommunication.endswitchesStates
        self.threadedManualDrive.sendPosition.connect(self.receiveCurrentPositionFromThread)
        self.threadedManualDrive.sendDirection.connect(self.changeDirectionPin)
        self.threadedManualDrive.moveToThread(self.manualDriveThread)
        self.manualDriveThread.started.connect(self.threadedManualDrive.goToPosition)
        QtWidgets.qApp.aboutToQuit.connect(self.manualDriveThread.quit)
        self.manualDriveThread.start()

    def performReferenceScan(self):
        self.referenceScanThread=QtCore.QThread()
        self.threadedReferenceScan=threadedTasks(self.cncObject.stepsPerMillimeter)
        self.threadedReferenceScan.currentPosition=self.cncObject.currentPosition
        self.threadedReferenceScan.speed=win.speedManualDrive.value()
        self.threadedReferenceScan.done.connect(self.referenceScanDone)
        self.threadedReferenceScan.sendPosition.connect(self.receiveCurrentPositionFromThread)
        self.threadedReferenceScan.sendCurrentProgramStep.connect(self.receiveCurrentProgramStep)
        self.threadedReferenceScan.speed=win.speedManualDrive.value()
        self.threadedReferenceScan.endswitchesStates=self.myRaspberryCommunication.endswitchesStates
        self.threadedReferenceScan.moveToThread(self.referenceScanThread)
        self.referenceScanThread.started.connect(self.threadedReferenceScan.performReferenceScan)
        QtWidgets.qApp.aboutToQuit.connect(self.referenceScanThread.quit)
        self.referenceScanThread.start()
        win.stopManualDrive.setEnabled(True)

    def receiveCurrentProgramStep(self,currentProgramStep):
        self.cncObject.currentProgramStep=self.cncObject.stepBeforePause+currentProgramStep
        progress=(self.cncObject.stepBeforePause+currentProgramStep+1)/self.cncObject.programLength*100
        win.progressBar.setValue(progress)
        win.progressValue.setText('{:.1f}%'.format(progress))

    def receiveCurrentPositionFromThread(self,x,y,z,drawPoint=False):
        win.currentXpos.setText('{:.2f} mm'.format(x))
        win.xPosValue.setText('{:.2f} mm'.format(x))
        win.currentYpos.setText('{:.2f} mm'.format(y))
        win.yPosValue.setText('{:.2f} mm'.format(y))
        win.currentZpos.setText('{:.2f} mm'.format(z))
        win.zPosValue.setText('{:.2f} mm'.format(z))

        self.cncObject.currentPosition['x']['mm']=x
        self.cncObject.currentPosition['y']['mm']=y
        self.cncObject.currentPosition['z']['mm']=z

        #The following is commented out because it slows down the programm
        """
        if drawPoint:
            self.alreadyProcessedSteps=np.vstack((self.alreadyProcessedSteps,np.array([x,y,z])))
            win.alreadProcessedStepsPlot.set_xdata(self.alreadyProcessedSteps[...,0])
            win.alreadProcessedStepsPlot.set_ydata(self.alreadyProcessedSteps[...,1])
            win.alreadProcessedStepsPlot.set_3d_properties(self.alreadyProcessedSteps[...,2])
            win.canvas2.draw()
            win.canvas2.flush_events()
        """

    def changeDirectionPin(self,axis,direction):
        print(axis, direction)

    def changeSpeed(self):
        if self.threadedProgram is not None:
            self.threadedProgram.speed=win.speed.value()

    def runProgram(self):
        self.myRaspberryCommunication.turnOnOffFan(1, 200)
        self.turnOnOffMillingMotor(1)
        time.sleep(2)
        self.isProgram=True
        win.runButton.setDisabled(True)
        win.pauseButton.setEnabled(True)
        win.stopButton.setEnabled(True)
        if not self.myRaspberryCommunication.endswitchesStates['allAxes']:
            win.progressBar.setValue((self.cncObject.stepBeforePause)/self.cncObject.programLength*100)

            self.programThread=QtCore.QThread()
            self.threadedProgram=threadedTasks(self.cncObject.stepsPerMillimeter)
            self.threadedProgram.currentPosition=self.cncObject.currentPosition
            if self.cncObject.programIsPaused:
                self.cncObject.stepBeforePause+=self.cncObject.currentProgramStep
                programCode=np.vstack((np.array([self.cncObject.currentPosition['x']['mm'],self.cncObject.currentPosition['y']['mm'],self.cncObject.currentPosition['z']['mm']]),self.cncObject.toolpath[self.cncObject.stepBeforePause:]))
            else:
                self.alreadyProcessedSteps=np.empty((0,3))
                programCode=self.cncObject.toolpath
            self.threadedProgram.points=programCode
            self.threadedProgram.done.connect(self.programDone)
            self.threadedProgram.sendDirection.connect(self.changeDirectionPin)
            self.threadedProgram.sendPosition.connect(self.receiveCurrentPositionFromThread)
            self.threadedProgram.sendCurrentProgramStep.connect(self.receiveCurrentProgramStep)
            self.threadedProgram.speed=win.speed.value()
            self.threadedProgram.moveToThread(self.programThread)
            self.programThread.started.connect(self.threadedProgram.runProgram)
            QtWidgets.qApp.aboutToQuit.connect(self.programThread.quit)
            self.programThread.start()

    def pauseProgram(self):
        self.myRaspberryCommunication.turnOnOffFan(1, 100)
        win.runButton.setEnabled(True)
        win.pauseButton.setDisabled(True)
        win.stopButton.setDisabled(True)
        self.cncObject.programIsPaused=True
        self.threadedProgram.pause()

    def stopProgram(self):
        win.runButton.setEnabled(True)
        win.pauseButton.setDisabled(True)
        win.stopButton.setDisabled(True)
        self.cncObject.programIsPaused=False
        self.cncObject.stepBeforePause=self.cncObject.currentProgramStep
        self.threadedProgram.stop()
        self.programThread.quit()
        self.threadedProgram=None
        self.turnOnOffMillingMotor(0, delay=1)

    def programDone(self):
        self.myRaspberryCommunication.turnOnOffFan(1, 100)
        self.turnOnOffMillingMotor(0)
        win.goToPosition.setEnabled(True)
        win.pauseButton.setDisabled(True)
        win.stopButton.setDisabled(True)
        win.stopManualDrive.setDisabled(True)
        win.runButton.setEnabled(True)
        self.programIsPaused=False
        self.cncObject.stepBeforePause=0

        self.threadedProgram.stop()
        self.programThread.quit()
        self.threadedProgram=None

    def stopManualDrive(self):
        win.joggPosXSlow.setEnabled(True)
        win.joggPosXFast.setEnabled(True)
        win.joggNegXSlow.setEnabled(True)
        win.joggNegXFast.setEnabled(True)
        win.joggPosYSlow.setEnabled(True)
        win.joggPosYFast.setEnabled(True)
        win.joggNegYSlow.setEnabled(True)
        win.joggNegYFast.setEnabled(True)
        win.joggPosZSlow.setEnabled(True)
        win.joggPosZFast.setEnabled(True)
        win.joggNegZSlow.setEnabled(True)
        win.joggNegZFast.setEnabled(True)
        win.goToPosition.setEnabled(True)
        win.goToPosition.setEnabled(True)
        win.stopManualDrive.setDisabled(True)
        if self.threadedManualDrive!=None:
            self.threadedManualDrive.stop()
            self.manualDriveThread.quit()
            self.threadedManualDrive=None
        elif self.referenceScanThread!=None:
            self.threadedReferenceScan.stop()
            self.referenceScanThread.quit()
            self.threadedReferenceScan=None

    def manualDriveDone(self):
        win.joggPosXSlow.setEnabled(True)
        win.joggPosXFast.setEnabled(True)
        win.joggNegXSlow.setEnabled(True)
        win.joggNegXFast.setEnabled(True)
        win.joggPosYSlow.setEnabled(True)
        win.joggPosYFast.setEnabled(True)
        win.joggNegYSlow.setEnabled(True)
        win.joggNegYFast.setEnabled(True)
        win.joggPosZSlow.setEnabled(True)
        win.joggPosZFast.setEnabled(True)
        win.joggNegZSlow.setEnabled(True)
        win.joggNegZFast.setEnabled(True)
        win.goToPosition.setEnabled(True)
        win.stopManualDrive.setDisabled(True)
        if self.threadedManualDrive is not None:
            self.threadedManualDrive.stop()
        self.manualDriveThread.quit()
        self.threadedManualDrive=None

    def referenceScanDone(self):
        win.goToPosition.setEnabled(True)
        win.stopManualDrive.setDisabled(True)
        if self.threadedManualDrive is not None:
            self.threadedReferenceScan.stop()
        self.referenceScanThread.quit()
        self.threadedReferenceScan=None

    def jogg(self, axis, steps, speed):
        self.programIsPaused=False
        self.cncObject.stepBeforePause=0
        if axis=='x':
            if ((self.myRaspberryCommunication.endswitchesStates['x']['front'] and steps<0) or (self.myRaspberryCommunication.endswitchesStates['x']['rear'] and steps>0)):
                return
            x=self.cncObject.currentPosition['x']['mm']+steps/self.cncObject.stepsPerMillimeter['x']
            y=self.cncObject.currentPosition['y']['mm']
            z=self.cncObject.currentPosition['z']['mm']
        elif axis=='y':
            if (((self.myRaspberryCommunication.endswitchesStates['y1']['front'] or self.myRaspberryCommunication.endswitchesStates['y1']['front']) and steps<0) or ((self.myRaspberryCommunication.endswitchesStates['y1']['rear'] or self.myRaspberryCommunication.endswitchesStates['y1']['rear']) and steps>0)):
                return
            x=self.cncObject.currentPosition['x']['mm']
            y=self.cncObject.currentPosition['y']['mm']+steps/self.cncObject.stepsPerMillimeter['y']
            z=self.cncObject.currentPosition['z']['mm']

        elif axis=='z':
            if ((self.myRaspberryCommunication.endswitchesStates['z']['front'] and steps<0) or (self.myRaspberryCommunication.endswitchesStates['z']['rear'] and steps>0)):
                return
            x=self.cncObject.currentPosition['x']['mm']
            y=self.cncObject.currentPosition['y']['mm']
            z=self.cncObject.currentPosition['z']['mm']+steps/self.cncObject.stepsPerMillimeter['z']
        self.goToPosition(x,y,z,manual=True)

    def toggleMillingMotor(self):
        self.millingMotorStatus=win.toggleMillingMotor.value()
        self.myRaspberryCommunication.changeMillingMotorStatus(self.millingMotorStatus)
        win.millingMotorLable.setText("on" if self.millingMotorStatus==1 else "off")


    def updateToggleSwitch(self):
        win.toggleMillingMotor.setProperty("value", self.millingMotorStatus)
        win.millingMotorLable.setText("on" if self.millingMotorStatus==1 else "off")

    def turnOnOffMillingMotor(self, status, delay=None):
        if delay is not None:
            time.sleep(delay)
        self.millingMotorStatus=status
        self.updateToggleSwitch()#the value change triggers toggleMillingMotor

class MyMainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent)
        QtWidgets.qApp.installEventFilter(self)
        self.setupUi(self)

        self.fig = Figure(dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.plot3d.addWidget(self.canvas)


        self.fig2 = Figure(dpi=100)
        self.canvas2 = FigureCanvas(self.fig2)
        self.canvas2.setParent(self)
        self.ax2 = self.fig2.add_subplot(111, projection='3d')
        self.fig2.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)
        self.ax2.set_xlabel('x')
        self.ax2.set_ylabel('y')
        self.ax2.set_zlabel('z')
        self.toolMesh,=self.ax2.plot([], [], [],'k-', zorder=150)
        self.plot3dProgramCode.addWidget(self.canvas2)
        self.canvas2.draw()

        self.myCncObject=myCNC()
        self.myStepperControl=stepperControl(self.myCncObject, self)

        self.stlImportButton.clicked[bool].connect(self.openStlFile)
        self.svgImportButton.clicked[bool].connect(self.openSvgFile)
        self.scaleSvg.valueChanged.connect(self.setSvgScaling)
        self.scaleStl.valueChanged.connect(self.setStlScaling)

        self.invertContour.setHidden(True)
        self.processingStrategy.currentIndexChanged.connect(self.setLabels)

        self.xOffsetLabel.setHidden(True)
        self.offsetX.setHidden(True)
        self.yOffsetLabel.setHidden(True)
        self.offsetY.setHidden(True)
        self.svgProcessingStrategy.currentIndexChanged.connect(self.setLabelsSvg)


        self.svgExportButton.clicked[bool].connect(self.saveSvgToNpzFile)
        self.stlExportButton.clicked[bool].connect(self.saveStlToNpzFile)
        self.generateSvgToolpath.clicked[bool].connect(self.myCncObject.generateSvgToolpath)
        self.generateStlToolpath.clicked[bool].connect(self.myCncObject.generateStlToolpath)

        self.loadConvertedFileButton.clicked[bool].connect(self.npzImport)

        self.pauseButton.clicked[bool].connect(self.myStepperControl.pauseProgram)
        self.stopButton.clicked[bool].connect(self.myStepperControl.stopProgram)
        self.runButton.clicked[bool].connect(self.myStepperControl.runProgram)
        self.pauseButton.setDisabled(True)
        self.stopButton.setDisabled(True)
        self.runButton.setDisabled(True)
        self.joggPosXSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('x',self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggPosXFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('x',self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.joggNegXSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('x',-self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggNegXFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('x',-self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.joggPosYSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('y',self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggPosYFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('y',self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.joggNegYSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('y',-self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggNegYFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('y',-self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.joggPosZSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('z',self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggPosZFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('z',self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.joggNegZSlow.clicked[bool].connect(lambda: self.myStepperControl.jogg('z',-self.myCncObject.slowStep, win.speedManualDrive.value()))
        self.joggNegZFast.clicked[bool].connect(lambda: self.myStepperControl.jogg('z',-self.myCncObject.fastStep, win.speedManualDrive.value()))
        self.speed.valueChanged.connect(self.myStepperControl.changeSpeed)


        self.setHome.clicked[bool].connect(self.myCncObject.setHomePosition)
        self.setZero.clicked[bool].connect(self.myCncObject.setZero)
        self.goHome.clicked[bool].connect(self.myStepperControl.goHome)
        self.performReferenceScan.clicked[bool].connect(self.myStepperControl.performReferenceScan)

        self.goToPosition.clicked[bool].connect(lambda: self.myStepperControl.goToPosition(self.xSetPoint.value(),self.ySetPoint.value(),self.zSetPoint.value(),manual=True))
        self.stopManualDrive.clicked[bool].connect(self.myStepperControl.stopManualDrive)
        self.stopManualDrive.setDisabled(True)

        self.toggleMillingMotor.valueChanged.connect(self.myStepperControl.toggleMillingMotor)

        self.canvasElements={}

        self.show()

    def setLabels(self):
        source = self.sender()
        if source.currentText()=='line':
            win.xPosLabel_9.setText('Slice Dist.')
            win.invertContour.setHidden(True)
        elif source.currentText()=='contour':
            win.xPosLabel_9.setText('Z depth')
            win.invertContour.setHidden(False)
        elif source.currentText()=='constant z':
            win.xPosLabel_9.setText('Z depth')
            win.invertContour.setHidden(False)

    def setLabelsSvg(self):
        source = self.sender()
        if source.currentText()=='contour':
            win.xOffsetLabel.setHidden(True)
            win.offsetX.setHidden(True)
            win.yOffsetLabel.setHidden(True)
            win.offsetY.setHidden(True)
        elif source.currentText()=='constant z':
            win.xOffsetLabel.setHidden(False)
            win.offsetX.setHidden(False)
            win.yOffsetLabel.setHidden(False)
            win.offsetY.setHidden(False)

    def openStlFile(self):
        source = self.sender()
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","STL files (*.stl)", options=options)
        if fileName:
            self.stlFilePathLabel.setText(fileName)
            source.setChecked(False)
            self.myCncObject.stlImport(fileName)

    def setStlScaling(self):
        source = self.sender()
        self.myCncObject.stlScaling=source.value()
        self.make3dPlot(self.canvas,self.ax)

    def openSvgFile(self):
        source = self.sender()
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","SVG files (*.svg)", options=options)
        if fileName:
            self.svgFilePathLabel.setText(fileName)
            source.setChecked(False)
            self.myCncObject.svgImport(fileName)

    def setSvgScaling(self):
        source = self.sender()
        self.myCncObject.svgScaling=source.value()
        self.make2dPlot(self.canvas,self.ax,scaling=self.myCncObject.svgScaling/constants.conversionFactorPPItoMM)

    def saveSvgToNpzFile(self):
        source = self.sender()
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,"QFileDialog.getSaveFileName()","","*.npz files (*.npz)", options=options)
        if fileName:
            source.setChecked(False)
            np.savez(fileName, type="2d",shape=self.rescaledContour,toolpath=self.myCncObject.toolpath, toolDiameter=self.toolDiameter.value())

    def saveStlToNpzFile(self):
        source = self.sender()
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,"QFileDialog.getSaveFileName()","","*.npz files (*.npz)", options=options)
        if fileName:
            source.setChecked(False)
            np.savez(fileName, type="3d",shape=self.myCncObject.stlScaling*self.myCncObject.facets,toolpath=self.myCncObject.toolpath,toolDiameter=self.toolDiameterStl.value())

    def make3dPlot(self,canvas, graph):
        graph.clear()
        self.myCncObject.translateToOrigin()

        mesh=mplot3d.art3d.Poly3DCollection(self.myCncObject.stlScaling*self.myCncObject.facets,facecolors=self.getFlatColorArray(),zsort='max')
        graph.add_collection3d(mesh)
        scale = self.myCncObject.stlScaling*self.myCncObject.facets.flatten("A")
        graph.auto_scale_xyz(scale, scale, scale)
        graph.set_xlabel('x')
        graph.set_ylabel('y')
        graph.set_zlabel('z')
        self.programCode,=graph.plot([],[],[],'r-', zorder=100)
        canvas.draw()

    def getFlatColorArray(self):
        colors=[]
        dark=[16,33,105]
        light=[50,91,255]
        minZvalue=self.myCncObject.stlScaling*min(self.myCncObject.facets[...,2][...,2])
        maxZvalue=self.myCncObject.stlScaling*max(self.myCncObject.facets[...,2][...,2])
        distance=maxZvalue-minZvalue
        for facet in self.myCncObject.stlScaling*self.myCncObject.facets:
            key=(np.mean(facet[...,2])-minZvalue)/distance
            colors.append([(dark[0]+(light[0]-dark[0])*key)/255,(dark[1]+(light[1]-dark[1])*key)/255,(dark[2]+(light[2]-dark[2])*key)/255,1])
        return colors

    def getColorArray(self):
        colors=[]
        dark=[16,33,105]
        light=[50,91,255]
        minZvalue=min(self.myCncObject.facets[...,2][...,2])
        maxZvalue=max(self.myCncObject.facets[...,2][...,2])
        distance=maxZvalue-minZvalue
        for facet in self.myCncObject.facets:
            for point in facet:
                facetCol=[]
                key=(point[2]-minZvalue)/distance
                facetCol.append([(dark[0]+(light[0]-dark[0])*key)/255,(dark[1]+(light[1]-dark[1])*key)/255,(dark[2]+(light[2]-dark[2])*key)/255,1])
                colors.append(facetCol)
        return colors

    def make2dPlot(self,canvas, graph, scaling=1):
        graph.clear()
        self.rescaledContour=[]
        for contour in self.myCncObject.importedContourForPlotting:
            self.rescaledContour.append(scaling*np.array(contour))
            graph.plot(scaling*np.array(contour)[:,0],scaling*np.array(contour)[:,1],scaling*np.array(contour)[:,2],'b-')
        graph.set_xlabel('x')
        graph.set_ylabel('y')
        graph.set_zlabel('z')
        self.programCode,=graph.plot([],[],[],'r-', zorder=100)

        canvas.draw()

    def plotToolpath(self,canvas,graph):
        self.programCode.set_xdata(self.myCncObject.toolpath[...,0])
        self.programCode.set_ydata(self.myCncObject.toolpath[...,1])
        self.programCode.set_3d_properties(self.myCncObject.toolpath[...,2])
        lowerLimit=min(min(self.myCncObject.toolpath[...,0])*1.1,min(self.myCncObject.toolpath[...,1])*1.1)
        upperLimit=max(max(self.myCncObject.toolpath[...,0])*1.1,max(self.myCncObject.toolpath[...,1])*1.1)
        graph.set_xlim(lowerLimit,upperLimit)
        graph.set_ylim(lowerLimit,upperLimit)
        graph.set_zlim(min(self.myCncObject.toolpath[...,2])*1.1,max(self.myCncObject.toolpath[...,2])*1.1)
        canvas.draw()

    def clearCanvasElements(self,canvas):
        keysToDelete=[]
        for elem in self.canvasElements:
            if 'grid' in elem:
                continue
            if self.canvasElements[elem]['canvas']!=canvas:
                continue
            if self.canvasElements[elem]['plot'] and type(self.canvasElements[elem]['plot']) is list:
                for contour in self.canvasElements[elem]['plot']:
                    self.canvasElements[elem]['canvas'].removeItem(contour)
            else:
                self.canvasElements[elem]['canvas'].removeItem(self.canvasElements[elem]['plot'])
            keysToDelete.append(elem)

        for elem in keysToDelete:
            del self.canvasElements[elem]

    def makeToolMesh(self):
        self.toolMesh.set_xdata([self.myCncObject.currentPosition['x']['mm'],self.myCncObject.currentPosition['x']['mm']])
        self.toolMesh.set_ydata([self.myCncObject.currentPosition['y']['mm'],self.myCncObject.currentPosition['y']['mm']])
        self.toolMesh.set_3d_properties([self.myCncObject.currentPosition['z']['mm'],self.myCncObject.currentPosition['z']['mm']+10])
        self.canvas2.draw()
        self.canvas2.flush_events()

    def npzImport(self):
        self.myCncObject.programIsPaused=False
        self.pauseButton.setDisabled(True)
        self.stopButton.setDisabled(True)
        self.runButton.setEnabled(True)
        source = self.sender()
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","*.npz files (*.npz)", options=options)

        self.myCncObject.svgScaling=1
        self.myCncObject.stlScaling=1

        if fileName:
            self.npzImportName.setText(fileName)
            source.setChecked(False)
            npzfile = np.load(fileName,allow_pickle=True)
            if npzfile['type']=="2d":
                self.myCncObject.importedContourForPlotting=npzfile['shape']
                self.make2dPlot(self.canvas2,self.ax2)

            if npzfile['type']=="3d":
                self.myCncObject.facets=npzfile['shape']
                self.make3dPlot(self.canvas2,self.ax2)
            self.myCncObject.toolpath=npzfile['toolpath']
            self.myCncObject.toolDiameter=npzfile['toolDiameter']
            self.myCncObject.programLength=len(self.myCncObject.toolpath)
            self.plotToolpath(self.canvas2,self.ax2)
            self.alreadProcessedStepsPlot,=self.ax2.plot([], [], [],'k-', zorder=150)

            self.toolMesh,=self.ax2.plot([], [], [],'k-', zorder=150)

#global is required
win=None

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MyMainWindow()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
