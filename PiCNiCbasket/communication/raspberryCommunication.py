from time import sleep
import importlib
import time
from datetime import datetime
import yaml

hasRPi = importlib.util.find_spec("RPi") is not None
hasAdafruit_I2C = importlib.util.find_spec("Adafruit_I2C")  is not None
hasMCP23017 = importlib.util.find_spec("MCP23017")  is not None
hasPIGPIO= importlib.util.find_spec("pigpio")  is not None

onRpi=hasRPi and hasPIGPIO

if onRpi:
    import pigpio
    import RPi.GPIO as GPIO
    from PiCNiCbasket.thirdParty.Adafruit_I2C import Adafruit_I2C
    from PiCNiCbasket.thirdParty.MCP23017 import MCP23017



class raspberryCommunication():
    def __init__(self):
        # Connect to gpiodd daemon
        if onRpi:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            self.mcp = MCP23017(address = 0x20, num_gpios = 16) # MCP23017
            self.MCPisConnected=False if self.mcp.i2c.readU8(0)==-1 else True
            self.pi = pigpio.pi()                        # initialize connection
            self.pi.wave_clear()
        self.pulseLength=0.001
        self.pinState=0
        self.pulseCount=0
        self.waveLength=0

        self.endswitchesStates={'allAxes':False,'x':{'front':False,'rear':False},'y1':{'front':False,'rear':False},'y2':{'front':False,'rear':False},'z':{'front':False,'rear':False}}
        self.microSteppingDict={1: (0, 0, 0),
              2: (1, 0, 0),
              4: (0, 1, 0),
              8: (1, 1, 0),
              16: (0, 0, 1),
              32: (1, 0, 1)}

        self.readConfigFile()
        self.configurePins()
        self.wave=None



    def configurePins(self):
        if onRpi:
            #milling motor relay
            GPIO.setup(self.pinNumber['millingMotorRelay'], GPIO.OUT)
            self.pi.set_mode(self.pinNumber['fan'], pigpio.OUTPUT)

            for axis in self.pinNumber['step']:
                GPIO.setup(self.pinNumber['step'][axis], GPIO.OUT)

            for axis in self.pinNumber['direction']:
                GPIO.setup(self.pinNumber['direction'][axis], GPIO.OUT)

            for axis in self.pinNumber['endswitches']:
                GPIO.setup(self.pinNumber['endswitches'][axis]['front'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                GPIO.setup(self.pinNumber['endswitches'][axis]['rear'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

            for axis in self.pinNumber['microStepping']:
                if self.pinNumber['microStepping'][axis]['isMCP'] and self.MCPisConnected:
                    self.mcp.pinMode(self.pinNumber['microStepping'][axis][0], self.mcp.OUTPUT)
                    self.mcp.pinMode(self.pinNumber['microStepping'][axis][1], self.mcp.OUTPUT)
                    self.mcp.pinMode(self.pinNumber['microStepping'][axis][2], self.mcp.OUTPUT)

                    self.mcp.output(self.pinNumber['microStepping'][axis][0], self.microSteppingDict[self.microStepping[axis]][0])
                    self.mcp.output(self.pinNumber['microStepping'][axis][1], self.microSteppingDict[self.microStepping[axis]][1])
                    self.mcp.output(self.pinNumber['microStepping'][axis][2], self.microSteppingDict[self.microStepping[axis]][2])
                else:
                    GPIO.setup(self.pinNumber['microStepping'][axis][0], GPIO.OUT)
                    GPIO.setup(self.pinNumber['microStepping'][axis][1], GPIO.OUT)
                    GPIO.setup(self.pinNumber['microStepping'][axis][2], GPIO.OUT)

                    GPIO.output(self.pinNumber['microStepping'][axis][0],self.microSteppingDict[self.microStepping[axis]][0])
                    GPIO.output(self.pinNumber['microStepping'][axis][1],self.microSteppingDict[self.microStepping[axis]][1])
                    GPIO.output(self.pinNumber['microStepping'][axis][2],self.microSteppingDict[self.microStepping[axis]][2])

    def readConfigFile(self):
        with open("config.yaml", "r") as yamlfile:
            config = yaml.load(yamlfile, Loader=yaml.FullLoader)
        self.pinNumber=config['pinNumbers']
        self.microStepping=config['microStepping']
        self.axisDirections=config['axisDirections']

    def getEndswitchesStates(self):
        if onRpi:
            for axis in self.pinNumber['endswitches']:
                if axis != 'allAxes':
                    self.endswitchesStates[axis]['front']=not GPIO.input(self.pinNumber['endswitches'][axis]['front'])
                    self.endswitchesStates[axis]['rear']=not GPIO.input(self.pinNumber['endswitches'][axis]['rear'])

            self.endswitchesStates['allAxes']=self.endswitchesStates['x']['front'] or self.endswitchesStates['x']['rear'] or self.endswitchesStates['y1']['front'] or self.endswitchesStates['y1']['rear'] or self.endswitchesStates['y2']['front'] or self.endswitchesStates['y2']['rear'] or  self.endswitchesStates['z']['front'] or self.endswitchesStates['z']['rear']

        return self.endswitchesStates

    def changeMillingMotorStatus(self, status):
        if onRpi:
            GPIO.output(self.pinNumber['millingMotorRelay'],status)

    def step(self,axis,direction):
        if onRpi:
            if axis=='y':
                GPIO.output(self.pinNumber['direction']['y1'], GPIO.HIGH if direction==-1*self.axisDirections['y1'] else GPIO.LOW)
                GPIO.output(self.pinNumber['direction']['y2'], GPIO.HIGH if direction==-1*self.axisDirections['y2'] else GPIO.LOW)

                GPIO.output(self.pinNumber['step']['y1'], GPIO.HIGH)
                GPIO.output(self.pinNumber['step']['y2'], GPIO.HIGH)
                sleep(self.pulseLength)
                GPIO.output(self.pinNumber['step']['y1'], GPIO.LOW)
                GPIO.output(self.pinNumber['step']['y2'], GPIO.LOW)
            else:
                output=GPIO.HIGH if direction==-1*self.axisDirections[axis] else GPIO.LOW
                GPIO.output(self.pinNumber['direction'][axis], output)
                GPIO.output(self.pinNumber['step'][axis], GPIO.HIGH)
                sleep(self.pulseLength)
                GPIO.output(self.pinNumber['step'][axis], GPIO.LOW)

    def checkEndswitches(self,deltaX,deltaY,deltaZ,isProgram):
        self.getEndswitchesStates()
        statusX=statusY=statusZ=False
        if not isProgram:
            if deltaY<0:
                statusX=self.endswitchesStates['y1']['front'] or self.endswitchesStates['y2']['front']
            elif deltaY>0:
                statusX=self.endswitchesStates['y1']['rear'] or self.endswitchesStates['y2']['rear']

            if deltaX<0:
                statusY=self.endswitchesStates['x']['front']
            elif deltaX>0:
                statusY=self.endswitchesStates['x']['rear']

            if deltaZ>0:
                statusZ=self.endswitchesStates['z']['rear']
            elif deltaZ<0:
                statusZ=self.endswitchesStates['z']['front']

            return statusX or statusY or statusZ
        return self.endswitchesStates['allAxes']

    def turnOnOffFan(self,status, speed):
        if onRpi:
            self.pi.set_PWM_frequency(self.pinNumber['fan'],200)
            self.pi.set_PWM_dutycycle(self.pinNumber['fan'], speed*status)

    def abortWave(self):
        self.pi.wave_tx_stop()
        self.pi.wave_delete(self.wave)
        self.pi.wave_clear()
        self.wave=None

    def pulseCounter(self,pin):
        currenState=self.pi.read(pin)
        if self.pinState!=currenState and currenState==1:
            self.pulseCount+=1
            print(self.pulseCount)
        self.pinState=currenState

    def waitForWaveToFinish(self, timeStart,executions, pulses):
        step=1
        while self.pi.wave_tx_busy():
            if self.checkEndswitches(self.deltaX,self.deltaY,self.deltaZ,self.isProgram) or self.controlClass.mainStopSignal:
                self.abortWave()
                timeNow=self.pi.get_current_tick()
                fraction=1-(executions*11000+(timeNow-timeStart)/self.waveLength*pulses)/self.totalPulses
                self.controlClass.currentPosition['x']['mm']=self.controlClass.targetPosition['x']-self.deltaX*fraction
                self.controlClass.currentPosition['y']['mm']=self.controlClass.targetPosition['y']-self.deltaY*fraction
                self.controlClass.currentPosition['z']['mm']=self.controlClass.targetPosition['z']-self.deltaZ*fraction
                return False
            if step%2000==0:
                timeNow=self.pi.get_current_tick()
                fraction=1-(executions*11000+(timeNow-timeStart)/self.waveLength*pulses)/self.totalPulses
                self.controlClass.sendPosition.emit(self.controlClass.targetPosition['x']-self.deltaX*fraction,self.controlClass.targetPosition['y']-self.deltaY*fraction,self.controlClass.targetPosition['z']-self.deltaZ*fraction,self.isProgram)
            step+=1
        return True

    def runSteps(self,pulseList,deltaX,deltaY,deltaZ,isProgram, controlClass):
        self.controlClass=controlClass
        self.deltaX=deltaX
        self.deltaY=deltaY
        self.deltaZ=deltaZ
        self.isProgram=isProgram
        self.totalPulses=len(pulseList)

        pulse_list=[]
        executions=0
        self.pulseCount=0
        pulses=0
        timeStart=self.pi.get_current_tick()
        for i,entry in enumerate(pulseList[:-1]):
            if entry[2]==1:
                pulse_list.append(pigpio.pulse(1<<entry[1], 0, int(pulseList[i+1][0]-entry[0])))
            else:
                pulse_list.append(pigpio.pulse(0, 1<<entry[1], int(pulseList[i+1][0]-entry[0])))

            if i%1000==0 and i>0: #submit waveform in junks of 1000 pulses
                pulses+=1000
                self.pi.wave_add_generic(pulse_list)
                pulse_list=[]
                pulse_list.append(pigpio.pulse(0, 0, self.pi.wave_get_micros()))
            if i%11000==0 and i>0: #submit waveform in junks of 11000 pulses
                #wait for last wave to finish
                if self.wave is not None:
                    if self.waitForWaveToFinish(timeStart,executions-1,11000)==False:
                        return False
                #increase execustions counter and reset pulses counter
                executions+=1
                pulses=0
                if self.wave is not None:
                    self.pi.wave_delete(self.wave)
                #create and submit new wave form
                self.pi.wave_add_generic(pulse_list)
                self.wave = self.pi.wave_create()
                #get wavelength and execute wave
                self.waveLength=self.pi.wave_get_micros()
                self.pi.wave_send_once(self.wave)
                timeStart=self.pi.get_current_tick()
                pulse_list=[]#reset pulselist

        #adds the last doubled zero step with zero length
        pulse_list.append(pigpio.pulse(0, 1<<pulseList[-1][1], 0))

        if self.wave is not None:
            if self.waitForWaveToFinish(timeStart,executions-1,11000)==False:
                return False
        #create last wave
        if self.wave is not None:
            self.pi.wave_delete(self.wave)
        self.pi.wave_add_generic(pulse_list)
        self.wave = self.pi.wave_create()
        #execute last wave
        self.waveLength=self.pi.wave_get_micros()
        self.pi.wave_send_once(self.wave)

        if self.waitForWaveToFinish(self.pi.get_current_tick(),executions,len(pulse_list)+pulses)==False:
            return False
        self.pi.wave_delete(self.wave)
        self.pi.wave_clear()
        self.wave=None
        return True
