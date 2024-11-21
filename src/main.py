from vex import *

# devices
brain = Brain()
controller = Controller()
leftDrive = Motor(Ports.PORT1, False)
rightDrive = Motor(Ports.PORT6, True)
intake1 = Motor(Ports.PORT9, False)
intake2 = Motor(Ports.PORT12, True)
flyWheel1 = Motor(Ports.PORT7, False)
flyWheel2 = Motor(Ports.PORT8, True)
loadedOptical = Optical(Ports.PORT2)
pneumatic1 = Pneumatic(Ports.PORT2)
pneumatic2 = Pneumatic(Ports.PORT10) 
Touch1 = Touchled(Ports.PORT12)
Touch2 = Touchled(Ports.PORT12)
intakeOptical = Optical(Ports.PORT12)
MainInertial = Inertial()
controller = Controller()

# device containers
driveTrain = SmartDrive(leftDrive, rightDrive, MainInertial)
flyWheel = MotorGroup(flyWheel1, flyWheel2)
intake = MotorGroup(intake1, intake2)

# coding variables
intakeStatus = "off"
flywheelStatus = "off"
slowmodeScale = 1
shutdownTime = 900
pumpStatus = "on"
shootingMode = "low"
flywheelMode = "fast"
shotBalls = 0
autoStopIntake = False

# pid stuff
desiredHeading = 0
currentHeading = 0
previous_error = 0
correction = 0
error = 0
intergral = 0
# TUNE THESE FOR STRAIGHT MOVEMENT

"""
TUNING GUIDE

KP: KP is how strongly the robot reacts to the error(how far the robot
is from the desired path)
If the KP is too low, the robot will not correct itself enough
If the KP is too high, the robot will overshoot correcting its self

KD: KD is to smooth out the corrections.

If the KD is too low, the robot may over correct
If the KD is too high, the robot will react too slowly

KI: Small details in the correction

If the KI is too low, it will start drifting after moving for a long time
If the KI is too high, the robot will become unstable, plus over correction
"""
Kp = 0.968
Ki = 0.1
Kd = 0.05

DEFAULTVELOCITY = 86.0025
PASSVELOCITY = 50

forwardInput = 0
turnInput = 0
derivative = 0
leftMotorSpeed = 0
rightMotorSpeed = 0
deadband = 20

# motor settings
flyWheel.set_max_torque(100, PERCENT)
flyWheel.set_velocity(DEFAULTVELOCITY, PERCENT)
intake.set_max_torque(100, PERCENT)
intake.set_velocity(100, PERCENT)
pneumatic1.pump_on()
pneumatic2.pump_on()
intakeOptical.set_light_power(0, PercentUnits.PERCENT)
intakeOptical.set_light(LedStateType.OFF)
loadedOptical.set_light_power(0, PercentUnits.PERCENT)
loadedOptical.set_light(LedStateType.OFF)

class ButtonBinding:
    button:Controller.Button
    callback:function

class ControllerLayout:
    def __init__(self, layoutTable:List, threadedList) -> None:
        self.layoutTable = layoutTable
        self.threadFunctionList = threadedList
        self.status = "inactive"
        self.connectionList = []
        self.threadFunctionList = threadedList
        self.threadList = []
    def AddLayout(self, binding:ButtonBinding) -> None:
        self.layoutTable.append(binding)

    def BindButtons(self):
        for buttonBinding in self.layoutTable:
            newBinding = buttonBinding.button.pressed(buttonBinding.callback)
            self.connectionList.append(newBinding)
        for threadFunction in self.threadFunctionList:
            newThread = Thread(threadFunction)
            self.threadList.append(newThread)

    layoutTable:List[ButtonBinding]
    connectionList:List[Event]
    threadFunctionList:List
    threadList:List[Thread]
    status:str
        

def healthCheckPneumatics():
    brain.screen.set_font(FontType.MONO20)
    if not pneumatic1.installed():
        brain.screen.print("PNEUMATIC NOT WORKING")
        while True:
            wait(0.1, SECONDS)


vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    print("calibrating")
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    MainInertial.calibrate()
    sleep(8,SECONDS)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)

calibrate_drivetrain()

def remoteControlLoop():
    global forwardInput, turnInput
    global Kp, Ki, Kd
    global intergral, previous_error, desiredHeading
    global currentHeading, desiredHeading
    global error
    global derivative
    global correction
    global leftMotorSpeed, rightMotorSpeed
    global deadband
    global slowmodeScale
    leftDrive.set_velocity(leftMotorSpeed, PERCENT)
    rightDrive.set_velocity(rightMotorSpeed, PERCENT)
    leftDrive.spin(FORWARD)
    rightDrive.spin(FORWARD)
    while True:
        forwardInput = (controller.axisA.position() ) * slowmodeScale
        turnInput = (controller.axisC.position()) * slowmodeScale * -1
        if abs(forwardInput) < deadband:
            forwardInput = 0
        
        if abs(turnInput) < deadband:
            turnInput = 0
        if turnInput == 0:
            currentHeading = MainInertial.heading()
            error = desiredHeading - currentHeading
            intergral = intergral + error
            derivative = error - previous_error
            correction = (Kp * error) + (Ki * intergral) + (Kd * derivative)
            leftMotorSpeed = forwardInput + correction
            rightMotorSpeed = forwardInput - correction
            leftDrive.set_velocity(leftMotorSpeed, PERCENT)
            rightDrive.set_velocity(rightMotorSpeed, PERCENT)
            previous_error = error
            leftDrive.spin(FORWARD)
            rightDrive.spin(FORWARD)
        else:
            leftMotorSpeed = forwardInput + turnInput
            rightMotorSpeed = forwardInput - turnInput
            desiredHeading = MainInertial.heading()
            leftDrive.set_velocity(leftMotorSpeed, PERCENT)
            rightDrive.set_velocity(rightMotorSpeed, PERCENT)
            leftDrive.spin(FORWARD)
            rightDrive.spin(FORWARD)
        sleep(10,MSEC)


def shootBall():
    # requires flywheel to be oin
    print("shooting ball")
    global flywheelStatus
    global shotBalls
    if flywheelStatus == "on":
        shotBalls += 1
        print(str(shotBalls))
        pneumatic1.retract(CylinderType.CYLINDER1)
        pneumatic2.retract(CylinderType.CYLINDER1)
        while controller.buttonLUp.pressing():
            wait(1,MSEC)
        pneumatic1.extend(CylinderType.CYLINDER1)
        pneumatic2.extend(CylinderType.CYLINDER1)
def elevateFlywheel():
    print("changing elevation")
    global shootingMode
    if shootingMode == "low":
        pneumatic1.extend(CylinderType.CYLINDER2)
        pneumatic2.extend(CylinderType.CYLINDER2)
        flyWheel.set_velocity(DEFAULTVELOCITY,PERCENT)
        shootingMode = "high"
    else:
        flyWheel.set_velocity((DEFAULTVELOCITY - 5),PERCENT)
        pneumatic1.retract(CylinderType.CYLINDER2)
        pneumatic2.retract(CylinderType.CYLINDER2)
        shootingMode = "low"

def printDebugBrainValues():
    global flywheelStatus
    global error, slowmodeScale
    brain.screen.set_font(FontType.MONO20)
    while True:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("F1: " + str(flyWheel1.velocity()))
        brain.screen.set_cursor(2, 1)
        brain.screen.print("F2: " + str(flyWheel2.velocity()))
        brain.screen.set_cursor(3, 1)
        brain.screen.print("FS: " + str(flywheelStatus))
        brain.screen.set_cursor(4, 1)
        brain.screen.print("ER: " + str(error))
        brain.screen.set_cursor(5,1)
        brain.screen.print("SC: " + str(slowmodeScale))
        wait(50, MSEC)

def toggleIntake():
    print("toggling intake")
    global intakeStatus 
    if intakeStatus == "off":
        intake.spin(FORWARD)
        intakeStatus = "in"
    elif intakeStatus == "in":
        intake.spin(REVERSE)
        intakeStatus = "out"
    elif intakeStatus == "out":
        intake.spin(FORWARD)
        intakeStatus = "in"


def stopIntake():
    global intakeStatus
    intake.stop()
    intakeStatus = "off"

def flywheelOff():
    flyWheel.stop()


def manualSpin():
    global flywheelStatus
    while controller.buttonEUp.pressing():
        flywheelStatus = "on"
        flyWheel.spin(FORWARD)
    flywheelStatus = "off"
    flyWheel.stop()


def togggleSlowMode():
    global slowmodeScale
    if slowmodeScale == 1:
        slowmodeScale = 0.48
    else:
        slowmodeScale = 1

def playSounds():
    while True:
        brain.play_sound(SoundType.ALARM)
        wait(5,MSEC)
def lockmotors():
    while True:
        leftDrive.stop()
        rightDrive.stop()
        flyWheel.stop()
        intake.stop()

def lockdown():
    Thread(playSounds)
    Thread(lockmotors)

def toggleFlyWheel():
    global flywheelStatus
    if flywheelStatus == "off":
        flywheelStatus = "on"
        flyWheel.spin(FORWARD)
    else:
        flywheelStatus = "off"
        flyWheel.stop()

def toggleFlywheelSpeed():
    global flywheelMode, flywheelStatus
    global DEFAULTVELOCITY, PASSVELOCITY
    if flywheelMode == "fast":
        flywheelMode = "slow"

        flyWheel.set_velocity(50,PERCENT)
    else:
        flywheelMode = "fast"
        flyWheel.set_velocity(DEFAULTVELOCITY,PERCENT)

def tickTimer():
    global shutdownTime
    while True:
        if shutdownTime > 0:
            shutdownTime -= 1
            wait(1,SECONDS)
        else:
            lockdown()

def toggleIntakeLock():
    global autoStopIntake
    autoStopIntake = not autoStopIntake


stopDebounce = False
def updateAutoLockIntake():
    global autoStopIntake
    global stopDebounce
    while True:
        if autoStopIntake:
            loadedOptical.set_light_power(100,PERCENT)
            intakeOptical.set_light_power(100,PERCENT)
            loadedOptical.set_light(LedStateType.ON)
            loadedOptical.set_light(LedStateType.ON)
            if loadedOptical.is_near_object() and intakeOptical.is_near_object():
                if stopDebounce == False:
                    stopDebounce = True
                    stopIntake()
                    while not intakeStatus == "in":
                        wait(10,MSEC)
                    stopDebounce = False
        else:
            loadedOptical.set_light_power(0,PERCENT)
            intakeOptical.set_light_power(0,PERCENT)
            loadedOptical.set_light(LedStateType.OFF)
            loadedOptical.set_light(LedStateType.OFF)

#comment out for tournements, dont want risk shutdown timer 

healthCheckPneumatics()
Thread(tickTimer)
Thread(updateAutoLockIntake)
Thread(remoteControlLoop)
Thread(printDebugBrainValues)
controller.buttonRUp.pressed(toggleIntake)
controller.buttonRDown.pressed(stopIntake)
controller.buttonLUp.pressed(shootBall)
controller.buttonLDown.pressed(elevateFlywheel)
controller.buttonEUp.pressed(toggleIntakeLock)
controller.buttonEDown.pressed(togggleSlowMode)
controller.buttonFUp.pressed(toggleFlywheelSpeed)
controller.buttonFDown.pressed(toggleFlyWheel)
