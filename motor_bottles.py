import motor_particles as mp
import brickpi
import motor_util
from motor_util import motors

def main():
    interface = brickpi.Interface()
    interface.initialize()

    motor_util.setUpInterface(interface)

    # Setup initial state of particles
    numberOfParticles = 100
    waypoint1_x = 84
    waypoint1_y = 30
    particles = [((waypoint1_x, waypoint1_y, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]
    hit = False

    mp.drawParticles(particles)

    # Assume robot at waypoint 0 : (84, 30)
    # Attack A
    print "ATTACK AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    (particles, hit) = mp.navigateToWaypoint(148, 40, particles, interface, motors)
    if not hit:
        (particles, hit) = mp.navigateToWaypoint(190, 45, particles, interface, motors)

    # Attack B
    print "RUSH BBBBBBBBBBBBBBBBBrBBBBBBBBBBBBBBBBB"
    #mp.navigateToWaypoint(146, 80, particles, interface, motors)# Mid point
    
    (particles, hit) = mp.navigateToWaypoint(126, 100, particles, interface, motors) 
    if not hit:
        (particles, hit) = mp.navigateToWaypoint(126, 145, particles, interface, motors) 
        if not hit: 
            (particles, hit) = mp.navigateToWaypoint(126, 190, particles, interface, motors)

    # Attack C
    print "ATTACK CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
    (particles, hit) = mp.navigateToWaypoint(100, 50, particles, interface, motors) # Mid point 
    if not hit:
        (particles, hit) = mp.navigateToWaypoint(42, 50, particles, interface, motors)   
        if not hit:
            (particles, hit) = mp.navigateToWaypoint(42, 100, particles, interface, motors)
            if not hit:
                (particles, hit) = mp.navigateToWaypoint(42, 155, particles, interface, motors)
    
    (particles, hit) = mp.navigateToWaypoint(84, 30, particles, interface, motors)
    

    interface.terminate()


if __name__ == "__main__":
    main()