import brickpi
import time
import math

rotate1deg = 6.571 / (math.pi/2)
rotateSensor1deg = 1.7 / (math.pi/2)
forward1cm = 15.3 / 40


# Sensor port values
touch_port_left  = 2
touch_port_right = 0
motors = [0,3]
motor_sensor = 2

def rotate(angle, interface, motors):
    motorRot = angle*rotate1deg
    interface.increaseMotorAngleReferences(motors,[motorRot,-motorRot])

    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
            time.sleep(0.1)


def rotate_deg(angle, interface, motors):
    rotate(angle * (math.pi/180), interface, motors)

# Returns true iff the touch sensors have been touched. Else false
def move(angle, interface, motors):
    interface.increaseMotorAngleReferences(motors,[angle,angle])
    initialAngle = interface.getMotorAngles(motors)[1][0]
    initialAngle0 = interface.getMotorAngles(motors)[0][0]

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)

        if angle > 0:
            touched = isTouched(interface)

            if touched:
                actual_distance = (motorAngles[1][0] - initialAngle)/forward1cm
                print "TOUCHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"
                stop(interface)
                return True, actual_distance

        if motorAngles :
            #print ((motorAngles[0][0] - initialAngle0)/forward1cm, (motorAngles[1][0] - initialAngle)/forward1cm)
            time.sleep(0.1)

    actual_distance = (motorAngles[1][0] - initialAngle)/forward1cm
    return False, actual_distance

def stop(interface):
    interface.setMotorPwm(motors[0],0)
    interface.setMotorPwm(motors[1],0)

def isTouched(interface):
    result_left  = interface.getSensorValue(touch_port_left)
    result_right = interface.getSensorValue(touch_port_right)

    touched_left  = False
    touched_right = False

    if result_left:
        touched_left  = result_left[0]
    if result_right:
        touched_right = result_right[0]


    return touched_left or touched_right


def set_pid(motorParams):
    motorParams.maxRotationAcceleration = 6.0
    motorParams.maxRotationSpeed = 12.0
    motorParams.feedForwardGain = 255/20.0
    motorParams.minPWM = 30.0
    motorParams.pidParameters.minOutput = -255
    motorParams.pidParameters.maxOutput = 255
    motorParams.pidParameters.k_p = 350
    motorParams.pidParameters.k_i = 450
    motorParams.pidParameters.k_d = 37

def set_pid_sensor(motorParams):
    motorParams.maxRotationAcceleration = 6.0
    motorParams.maxRotationSpeed = 12.0
    motorParams.feedForwardGain = 255/20.0
    motorParams.minPWM = 30.0
    motorParams.pidParameters.minOutput = -255
    motorParams.pidParameters.maxOutput = 255
    motorParams.pidParameters.k_p = 305
    motorParams.pidParameters.k_i = 150
    motorParams.pidParameters.k_d = 20


def rotateLeft90deg(interface, motors):
    rotate(math.pi/2, interface, motors)


def rotateRight90deg(interface, motors):
    rotate(-math.pi/2, interface, motors)

# Returns (hit, distance actually moved)
def forward(distance, interface, motors):
    (hit, actual_distance) = move(distance*forward1cm, interface, motors)
    #print "EXPECT: " + str(distance) + "    ACTUAL: " + str(actual_distance)
    return (hit, actual_distance)

def setUpInterface(interface):
    # Setup motors
    speed = 5
    interface.motorEnable(motors[0])
    interface.motorEnable(motors[1])
    interface.motorEnable(motor_sensor)
    motorParams = interface.MotorAngleControllerParameters()
    sensorMotorParams = interface.MotorAngleControllerParameters()
    set_pid(motorParams)
    set_pid_sensor(sensorMotorParams)
    interface.setMotorAngleControllerParameters(motors[0], motorParams)
    interface.setMotorAngleControllerParameters(motors[1], motorParams)
    interface.setMotorAngleControllerParameters(motor_sensor, sensorMotorParams)

    interface.sensorEnable(touch_port_left, brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(touch_port_right, brickpi.SensorType.SENSOR_TOUCH)

def rotate_sensor(interface, angle):
    assert angle < 2*math.pi
    motorRot = angle*rotateSensor1deg
    interface.increaseMotorAngleReferences([motor_sensor],[-motorRot])

    while not interface.motorAngleReferencesReached([motor_sensor]) :
        motorAngles = interface.getMotorAngles([motor_sensor])
        if motorAngles :
            time.sleep(0.1)

def rotate_sensor_deg(interface, angle):
    rotate_sensor(interface, angle*(math.pi/180))

