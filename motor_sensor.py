from motor_particles import getSonarValue
import brickpi
import motor_util
from motor_util import motors
import time

interface = brickpi.Interface()
interface.initialize()

motor_util.setUpInterface(interface)

motor_util.rotate_sensor_deg(interface, -45)
motor_util.rotate_sensor_deg(interface, -45)

for i in range(18):
    motor_util.rotate_sensor_deg(interface, 10)
    print "SONAR: " + str(getSonarValue(interface))
    time.sleep(0.5)
    
motor_util.rotate_sensor_deg(interface, -90)