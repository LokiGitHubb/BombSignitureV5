from vex import *

# devices
brain = Brain()
controller = Controller()
leftDrive = Motor(Ports.PORT6, True)
rightDrive = Motor(Ports.PORT1, False)
intake1 = Motor(Ports.PORT1, True)
intake2 = Motor(Ports.PORT1, False)
flyWheel1 = Motor(Ports.PORT1, False)
flyWheel2 = Motor(Ports.PORT1, True)
loadedOptical = Optical(Ports.PORT1)
pneumatic1 = Pneumatic(Ports.PORT6)
pneumatic2 = Pneumatic(Ports.PORT12)
Touch1 = Touchled(Ports.PORT1)
Touch2 = Touchled(Ports.PORT1)
intakeOptical = Optical(Ports.PORT1)
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
Kp = 1.0
Ki = 0.1
Kd = 0.05

forwardInput = 0
turnInput = 0
derivative = 0
leftMotorSpeed = 0
rightMotorSpeed = 0

# motor settings
flyWheel.set_max_torque(100, PERCENT)
flyWheel.set_velocity(100, PERCENT)
intake.set_max_torque(100, PERCENT)
intake.set_velocity(100, PERCENT)
pneumatic1.pump_on()
pneumatic2.pump_on
intakeOptical.set_light_power(100, PercentUnits.PERCENT)
intakeOptical.set_light(LedStateType.ON)
loadedOptical.set_light_power(100, PercentUnits.PERCENT)
loadedOptical.set_light(LedStateType.ON)

def healthCheckPneumatics():
    brain.screen.set_font(FontType.MONO20)
    if not pneumatic1.installed():
        brain.screen.print("PNEUMATIC NOT WORKING")
        while True:
            wait(0.1, SECONDS)


def calibrateInertial():
    brain.screen.print("Calibrating Brain")
    MainInertial.calibrate()
    wait(10,SECONDS)
    brain.screen.clear_screen()


def remoteControlLoop():
    global forwardInput, turnInput
    global Kp, Ki, Kd
    global intergral, previous_error, desiredHeading
    global currentHeading, desiredHeading
    global error
    global derivative
    global correction
    global leftMotorSpeed, rightMotorSpeed
    global slowmodeScale
    leftDrive.set_velocity(leftMotorSpeed, PERCENT)
    rightDrive.set_velocity(rightMotorSpeed, PERCENT)
    leftDrive.spin(FORWARD)
    rightDrive.spin(FORWARD)
    while True:
        forwardInput = (controller.axisA.position() * 100) * slowmodeScale
        turnInput = (controller.axisC.position() * 100) * slowmodeScale
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


def shootBall():
    # requires flywheel to be oin
    global flywheelStatus
    if flywheelStatus == "on":
        pneumatic1.retract(CylinderType.CYLINDER1)
        pneumatic2.retract(CylinderType.CYLINDER2)
        while controller.buttonLUp.pressing():
            wait(1,MSEC)
        pneumatic1.retract(CylinderType.CYLINDER1)
        pneumatic2.retract(CylinderType.CYLINDER2)


def updateFlyWheelStatus():
    global flywheelStatus
    while True:
        if intakeOptical.is_near_object() or loadedOptical.is_near_object():
            flywheelStatus = "on"
            flyWheel.spin(FORWARD)
        else:
            flywheelStatus = "off"
            flyWheel.stop()


def printDebugBrainValues():
    global flywheelStatus
    brain.screen.set_font(FontType.MONO20)
    while True:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("F1: " + str(flyWheel1.velocity()))
        brain.screen.set_cursor(2, 1)
        brain.screen.print("F2: " + str(flyWheel2.velocity()))
        brain.screen.set_cursor(3, 1)
        brain.screen.print("FS: " + str(flywheelStatus))
        wait(50, MSEC)

def checkBallInIntake():
    if intakeOptical.is_near_object():
        if loadedOptical.is_near_object():
            intake.stop()

def toggleIntake():
    global intakeStatus
    if intakeStatus == "off":
        intake.spin(FORWARD)
        intakeStatus = "in"
    elif intakeStatus == "in":
        intake.spin(REVERSE)
        intakeStatus = "out"
    elif intakeStatus == "out":
        intake.spin(FORWARD)


def stopIntake():
    global intakeStatus
    intake.stop()
    intakeStatus = "off"

def togglePump():
    global pumpStatus
    if pumpStatus == "on":
        pneumatic1.pump_off()
        pneumatic2.pump_off()

        pumpStatus = "off"
    else:
        pumpStatus = "on"
        pneumatic1.pump_off()
        pneumatic2.pump_on()


def flywheelOff():
    flyWheel.stop()


def manualSpin():
    while controller.buttonEUp.pressing():
        flyWheel.spin(FORWARD)
    flyWheel.stop()


def togggleSlowMode():
    global slowmodeScale
    if slowmodeScale == 1:
        slowmodeScale = 0.5
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

def tickTimer():
    global shutdownTime
    while True:
        if shutdownTime > 0:
            shutdownTime -= 1
            wait(1,SECONDS)
        else:
            lockdown()
#comment out for tournements, dont want risk screwing

#healthCheckPneumatics()
calibrateInertial()
Thread(tickTimer)
Thread(remoteControlLoop)
Thread(updateFlyWheelStatus)
Thread(printDebugBrainValues)
controller.buttonRUp.pressed(toggleIntake)
controller.buttonRDown.pressed(stopIntake)
controller.buttonLUp.pressed(shootBall)
controller.buttonLDown.pressed(flywheelOff)
controller.buttonFUp.pressed(togglePump)
controller.buttonEUp.pressed(manualSpin)
controller.buttonEDown.pressed(togggleSlowMode)
