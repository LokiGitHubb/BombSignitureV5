from vex import *

# devices
brain = Brain()
controller = Controller()
leftDrive = Motor(Ports.PORT1, True)
rightDrive = Motor(Ports.PORT1, False)
intake1 = Motor(Ports.PORT1, True)
intake2 = Motor(Ports.PORT1, False)
flyWheel1 = Motor(Ports.PORT1, False)
flyWheel2 = Motor(Ports.PORT1, True)
topSensor = Optical(Ports.PORT1)
Touch1 = Touchled(Ports.PORT1)
Touch2 = Touchled(Ports.PORT1)
MainInertial = Inertial()
controller = Controller()

# device containers
driveTrain = SmartDrive(leftDrive, rightDrive, MainInertial)
flyWheel = MotorGroup(flyWheel1, flyWheel2)
intake = MotorGroup(intake1, intake2)

# coding variables
intakeStatus = "off"
flywheelStatus = "off"

# pid stuff
desiredHeading = 0
currentHeading = 0
previous_error = 0
correction = 0
error = 0
intergral = 0
Kp = 1.0
Ki = 0.1
Kd = 0.05
forwardInput = 0
turnInput = 0
derivative = 0
leftMotorSpeed = 0
rightMotorSpeed = 0


def calibrateInertial():
    brain.screen.print("Calibrating Brain")
    MainInertial.calibrate()
    while MainInertial.is_calibrating():
        wait(50, MSEC)
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
    leftDrive.set_velocity(leftMotorSpeed, PERCENT)
    rightDrive.set_velocity(rightMotorSpeed, PERCENT)
    leftDrive.spin(FORWARD)
    rightDrive.spin(FORWARD)
    while True:
        forwardInput = controller.axisA.position()
        turnInput = controller.axisC.position()
        if turnInput == 0:
            currentHeading = MainInertial.heading()
            error = desiredHeading - currentHeading
            intergral = intergral + error
            derivative = error - previous_error
            correction = (Kp * error) + (Ki * intergral) + (Kd * derivative)
            leftMotorSpeed = forwardInput + correction
            rightMotorSpeed = forwardInput + correction
            leftDrive.set_velocity(leftMotorSpeed, PERCENT)
            rightDrive.set_velocity(rightMotorSpeed, PERCENT)
            previous_error = error
        else:
            leftMotorSpeed = forwardInput + correction
            rightMotorSpeed = forwardInput + correction
            desiredHeading = MainInertial.heading()
            leftDrive.set_velocity(leftMotorSpeed, PERCENT)
            rightDrive.set_velocity(rightMotorSpeed, PERCENT)

calibrateInertial()
Thread(remoteControlLoop)
