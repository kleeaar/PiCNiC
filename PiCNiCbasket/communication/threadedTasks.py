from PyQt5 import QtCore, QtGui, QtWidgets
from PiCNiCbasket.communication.raspberryCommunication import raspberryCommunication
import numpy as np
import time


class threadedTasks(QtCore.QObject):
    done = QtCore.pyqtSignal()
    sendPosition = QtCore.pyqtSignal(float, float, float,bool)
    sendCurrentProgramStep = QtCore.pyqtSignal(int)
    sendDirection = QtCore.pyqtSignal(str, int)

    def __init__(self,stepsPerMillimeter):
        super().__init__()
        self.myRaspberryCommunication=raspberryCommunication()
        self.stepsPerMillimeter=stepsPerMillimeter
        self.currentPosition={}
        self.targetPosition={}
        self.mainStopSignal=False
        self.pauseSignal=False
        self.speed=0
        self.points=[]
        self.velocity=[]
        self.endswitchesStates=self.myRaspberryCommunication.getEndswitchesStates()
        self.stepsFractionX=0
        self.stepsFractionY=0
        self.stepsFractionZ=0

    @QtCore.pyqtSlot()
    def runProgram(self):
        for programStep,point in enumerate(self.points):
            if self.mainStopSignal or self.pauseSignal:
                self.sendCurrentProgramStep.emit(programStep)
                break
            self.targetPosition['x']=point[0]
            self.targetPosition['y']=point[1]
            self.targetPosition['z']=point[2]
            if not self.goToPosition(velocity=self.velocity[programStep],isProgram=True):
                break #endswitch triggered => abort program
            self.sendCurrentProgramStep.emit(programStep)
        if not self.mainStopSignal:
            self.myRaspberryCommunication.pi.stop()
            self.done.emit()

    def performReferenceScan(self):
        if not self.endswitchesStates['z']['front']:
            self.targetPosition['x']=self.currentPosition['x']['mm']
            self.targetPosition['y']=self.currentPosition['y']['mm']
            self.targetPosition['z']=500
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.targetPosition['z']=self.currentPosition['z']['mm']-4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed/=10.
            self.targetPosition['z']=self.currentPosition['z']['mm']+4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed*=10.
        if not (self.endswitchesStates['x']['front']):
            self.targetPosition['x']=-500
            self.targetPosition['y']=self.currentPosition['y']['mm']
            self.targetPosition['z']=self.currentPosition['z']['mm']
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.targetPosition['x']=self.currentPosition['x']['mm']+4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed/=10.
            self.targetPosition['x']=self.currentPosition['x']['mm']-4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed*=10.
        if not self.endswitchesStates['y1']['front'] or self.endswitchesStates['y2']['front']:
            self.targetPosition['x']=self.currentPosition['x']['mm']
            self.targetPosition['y']=-500
            self.targetPosition['z']=self.currentPosition['z']['mm']
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.targetPosition['y']=self.currentPosition['y']['mm']+4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed/=10.
            self.targetPosition['y']=self.currentPosition['y']['mm']-4.0
            if not self.mainStopSignal:
                self.goToPosition(isReferenceScan=True)
            self.speed*=10.
        self.myRaspberryCommunication.pi.stop()
        self.done.emit()

    def goToPosition(self, velocity=0, isProgram=False, isReferenceScan=False):
        deltaX=self.targetPosition['x']-self.currentPosition['x']['mm']
        deltaY=self.targetPosition['y']-self.currentPosition['y']['mm']
        deltaZ=self.targetPosition['z']-self.currentPosition['z']['mm']

        dirX=np.sign(deltaX)
        dirY=np.sign(deltaY)
        dirZ=np.sign(deltaZ)

        tx,ty,tz=[],[],[]

        pulseLength=150

        nStepsX=np.abs(int(self.stepsPerMillimeter['x']*deltaX+self.stepsFractionX))
        nStepsY=np.abs(int(self.stepsPerMillimeter['y']*deltaY+self.stepsFractionY))
        nStepsZ=np.abs(int(self.stepsPerMillimeter['z']*deltaZ+self.stepsFractionZ))
        self.stepsFractionX=self.stepsPerMillimeter['x']*deltaX+self.stepsFractionX-int(self.stepsPerMillimeter['x']*deltaX+self.stepsFractionX)
        self.stepsFractionY=self.stepsPerMillimeter['y']*deltaY+self.stepsFractionY-int(self.stepsPerMillimeter['y']*deltaY+self.stepsFractionY)
        self.stepsFractionZ=self.stepsPerMillimeter['z']*deltaZ+self.stepsFractionZ-int(self.stepsPerMillimeter['z']*deltaZ+self.stepsFractionZ)

        velocityMultiplier=1 if velocity==0 else 3
        disableAcceleration=True
        accelerationDuration=1000000.#in us
        duration=max(np.abs([deltaX,deltaY,deltaZ]))/(self.speed*velocityMultiplier)*1e6 #in us
        if duration!=0:
            if np.abs(deltaZ)/duration*1e6>self.myRaspberryCommunication.maxVelocity['z']: # limit z-axis speed
                duration=np.abs(deltaZ)/self.myRaspberryCommunication.maxVelocity['z']*1e6
            elif np.abs(deltaX)/duration*1e6>self.myRaspberryCommunication.maxVelocity['x'] or np.abs(deltaY)/duration*1e6>self.myRaspberryCommunication.maxVelocity['y']: # limit x/y-axis speed
                if np.abs(deltaX)/self.myRaspberryCommunication.maxVelocity['x']>np.abs(deltaY)/self.myRaspberryCommunication.maxVelocity['y']:#x-axis defines the duration of the motion -> y-axis is slower
                    if np.abs(deltaX)/duration*1e6>self.myRaspberryCommunication.maxVelocity['x']:
                        duration=np.abs(deltaX)/self.myRaspberryCommunication.maxVelocity['x']*1e6
                else:
                    if np.abs(deltaY)/duration*1e6>self.myRaspberryCommunication.maxVelocity['y']:
                        duration=np.abs(deltaY)/self.myRaspberryCommunication.maxVelocity['y']*1e6

        for i in range(nStepsX):
            if isProgram or disableAcceleration:
                startTime=i*duration/float(nStepsX)+10#delay by 10us to direction pin
            else:
                x=(i*duration/float(nStepsX)/accelerationDuration)
                accelTime=np.log(x/0.1+1)+x
                startTime=accelTime*accelerationDuration+10#delay by 10us to direction pin
            tx.append([startTime,self.myRaspberryCommunication.pinNumber['step']['x'],1])
            tx.append([startTime+pulseLength,self.myRaspberryCommunication.pinNumber['step']['x'],0])
        for i in range(nStepsY):
            if isProgram or disableAcceleration:
                startTime=i*duration/float(nStepsY)+10
            else:
                x=(i*duration/float(nStepsY)/accelerationDuration)
                accelTime=np.log(x/0.1+1)+x
                startTime=accelTime*accelerationDuration+10#delay by 10us to direction pin

            ty.append([startTime,self.myRaspberryCommunication.pinNumber['step']['y1'],1])
            ty.append([startTime,self.myRaspberryCommunication.pinNumber['step']['y2'],1])
            ty.append([startTime+pulseLength,self.myRaspberryCommunication.pinNumber['step']['y1'],0])
            ty.append([startTime+pulseLength,self.myRaspberryCommunication.pinNumber['step']['y2'],0])
        for i in range(nStepsZ):
            if isProgram or disableAcceleration:
                startTime=i*duration/float(nStepsZ)+10
            else:
                x=(i*duration/float(nStepsZ)/accelerationDuration)
                accelTime=np.log(x/0.1+1)+x
                startTime=accelTime*accelerationDuration+10#delay by 10us to direction pin
            tz.append([startTime,self.myRaspberryCommunication.pinNumber['step']['z'],1])
            tz.append([startTime+pulseLength,self.myRaspberryCommunication.pinNumber['step']['z'],0])

        #add last another step at zero, so last step is not too short
        #only correct for non acceleration case
        if nStepsX>0:
            tx.append([(float(nStepsX)+1)*duration/float(nStepsX)+10,self.myRaspberryCommunication.pinNumber['step']['x'],0])
        if nStepsY>0:
            ty.append([(float(nStepsY)+1)*duration/float(nStepsY)+10,self.myRaspberryCommunication.pinNumber['step']['y1'],0])
            ty.append([(float(nStepsY)+1)*duration/float(nStepsY)+10,self.myRaspberryCommunication.pinNumber['step']['y2'],0])

        if nStepsZ>0:
            tz.append([(float(nStepsZ)+1)*duration/float(nStepsZ)+10,self.myRaspberryCommunication.pinNumber['step']['z'],0])

        mergedSteps=[[0,self.myRaspberryCommunication.pinNumber['direction']['x'],1 if dirX==-1*self.myRaspberryCommunication.axisDirections['x'] else 0],
                    [0,self.myRaspberryCommunication.pinNumber['direction']['y1'],1 if dirY==-1*self.myRaspberryCommunication.axisDirections['y1'] else 0],
                    [0,self.myRaspberryCommunication.pinNumber['direction']['y2'],1 if dirY==-1*self.myRaspberryCommunication.axisDirections['y2'] else 0],
                    [0,self.myRaspberryCommunication.pinNumber['direction']['z'],1 if dirZ==-1*self.myRaspberryCommunication.axisDirections['z'] else 0]] #direction values

        mergedSteps.extend(tx)
        mergedSteps.extend(ty)
        mergedSteps.extend(tz)

        mergedSteps.sort(key=lambda x: x[0])

        if not self.myRaspberryCommunication.runSteps(mergedSteps,deltaX,deltaY,deltaZ,isProgram,self):
            self.sendPosition.emit(self.currentPosition['x']['mm'],self.currentPosition['y']['mm'],self.currentPosition['z']['mm'],isProgram)
            return False #endswitches triggered => abort program

        self.currentPosition['x']['mm']=self.targetPosition['x']
        self.currentPosition['y']['mm']=self.targetPosition['y']
        self.currentPosition['z']['mm']=self.targetPosition['z']

        self.sendPosition.emit(self.targetPosition['x'],self.targetPosition['y'],self.targetPosition['z'],isProgram)
        if not isProgram and not isReferenceScan:
            self.myRaspberryCommunication.pi.stop()
            self.done.emit()
        return True

    @QtCore.pyqtSlot()
    def stop(self):
        self.mainStopSignal=True

    def pause(self):
        self.pauseSignal=True
