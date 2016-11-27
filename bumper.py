import brickpi
import time
import motor_util

interface=brickpi.Interface()
#interface.startLogging('logfile.txt')
interface.initialize()

# Setup motors
motors = [0,3]
speed = 5
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
motorParams = interface.MotorAngleControllerParameters()
motor_util.set_pid(motorParams)
interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

# Setup sensors
touch_port_left  = 2
touch_port_right = 0
interface.sensorEnable(touch_port_left, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_port_right, brickpi.SensorType.SENSOR_TOUCH)

while (True):
    motor_util.move(10, interface, motors)
    
    result_left  = interface.getSensorValue(touch_port_left)
    result_right = interface.getSensorValue(touch_port_right)
    touched_left  = False
    touched_right = False
    if result_left:
        touched_left  = result_left[0]
    if result_right:
        touched_right = result_right[0]

    if touched_left and touched_right:
        print "Bumper on"
        motor_util.move(-40, interface, motors)
        motor_util.rotateRight90deg(interface, motors)

    
    if touched_left: # turn
               motor_util.move(-40, interface, motors)
               motor_util.rotateRight90deg(interface, motors)

    if touched_right: # turn
               motor_util.move(-40, interface, motors)
               motor_util.rotateLeft90deg(interface, motors)


    



#motor_util.rotateRight90deg(interface, motors)
    
print "Finished."

#interface.stopLogging('logfile.txt')
interface.terminate()
