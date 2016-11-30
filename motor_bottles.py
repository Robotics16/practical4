import motor_particles as mp
import brickpi
import motor_util
import math
from motor_util import motors

scan_enable = True
particles = []
numberOfParticles = 100
waypoint1_x = 84
waypoint1_y = 30
interface = None

def attack_point(x, y, max_d = -1):
    global particles
    (particles, hit) = mp.navigateToWaypoint(x, y, particles, interface, motors, max_d)
    if not hit:
        if scan_enable: 
            (particles, hit) = mp.scan_area(particles, interface, motors)
    return (particles, hit)

def main():
    global interface
    interface = brickpi.Interface()
    interface.initialize()

    # Setup initial state of particles
    global particles
    particles = [((waypoint1_x, waypoint1_y, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]
    
    motor_util.setUpInterface(interface)
    mp.drawParticles(particles)
    
    # Simplify Form /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    #def notHitScan():
    #    if not hit:
    #        (particles, hit) = mp.scan_area(particles, interface, motors)
    #        return True
    #    return False
    
    #def notHitNavi(x, y, max_d):
    #    if not hit:
    #        (particles, hit) = mp.navigateToWaypoint(x, y, particles, interface, motors, max_d)
    #        return True
    #    return False
    
    #def navi(x, y, max_d):
    #    (particles, hit) = mp.navigateToWaypoint(x, y, particles, interface, motors, max_d) 
    
    #max_d_A = 30
    #max_d_B = 15
    #max_d_C = -1
    
    # Assume robot at waypoint 0 : (84, 30)
    # Attack A
    #print "ATTACK AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    
    #navi(148, 40, max_d_A)
    #if notHitScan():
    #    if notHitNavi(190, 45, max_d_A):
    #        if notHitScan():
    #            if not hit:
    #                print " :( CANT FIND BOTTLE A "

    # Attack B
    #print "RUSH BBBBBBBBBBBBBBBBBrBBBBBBBBBBBBBBBBB
    #navi(126, 70, max_d_B)
    #navi(148, 100, max_d_B)
    #if notHitScan():
    #    if notHitNavi(126, 145, max_d_B)
    #        if notHitScan():
    #            if notHitNavi(126, 190, max_d_C)
    #                if notHitScan():
    #                    if not hit: 
    #                        print " :( CANT FIND BOTTLE B "
    # Attack C
    #print "ATTACK CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
    #navi(100, 50, -1)
    #if notHitNavi(42, 50, max_d_C):
    #    if notHitScan():
    #        if notHitNavi(42, 100, max_d_C):
    #            if notHitScan():
    #               if notHitNavi(42, 155, max_d_C):
    #                    if notHitScan():
    #                        if not hit: 
    #                            print " :( CANT FIND BOTTLE C "
    
    #navi(84, 30, max_d_C)
    
    
    
    
    
    # ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    # Assume robot at waypoint 0 : (84, 30)
    # Attack A
    print "ATTACK AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    (particles, hit) = mp.navigateToWaypoint(150, 60, particles, interface, motors)
    if not hit:
        motor_util.rotate_deg(-75, interface, motors)
        particles = mp.updateParticlesRotate(particles, -75 * (math.pi/180))
        if scan_enable: 
            (particles, hit) = mp.scan_area(particles, interface, motors)
            
    if not hit:
        (particles, hit) = attack_point(145, 30)
        if not hit:
            (particles, hit) = attack_point(170, 30)
            if not hit:
                (particles, hit) = attack_point(180, 60)
                if not hit:
                    (particles, hit) = attack_point(190, 30)
                    if not hit:
                        print " :( CANT FIND BOTTLE A "    

    # Attack B
    print "RUSH BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
    (particles, hit) = mp.navigateToWaypoint(115, 60, particles, interface, motors)# Mid point NOTE: CHECK IF HIT C
    
    (particles, hit) = attack_point(150, 110)
    if not hit:
        (particles, hit) = attack_point(150, 145)
        if not hit:
            (particles, hit) = attack_point(150, 185)
            if not hit:
                (particles, hit) = attack_point(100, 185)
                if not hit:
                    (particles, hit) = attack_point(130, 160)
                    if hit:
                        (particles, hit) = mp.navigateToWaypoint(135, 85, particles, interface, motors)
                        (particles, hit) = mp.navigateToWaypoint(135, 100, particles, interface, motors)
                    if not hit:
                        (particles, hit) = attack_point(100, 160)
                        if not hit:
                            (particles, hit) = attack_point(100, 84)
                            if hit:
                                (particles, hit) = mp.navigateToWaypoint(135, 85, particles, interface, motors)
                                (particles, hit) = mp.navigateToWaypoint(135, 100, particles, interface, motors)
                            if not hit:
                                print " :( CANT FIND BOTTLE B "  
    # Attack C
    print "ATTACK CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
    (particles, hit) = mp.navigateToWaypoint(50, 55, particles, interface, motors) # Mid point 
    
    if not hit:
        (particles, hit) = attack_point(50, 110)
        if not hit:
            (particles, hit) = attack_point(50, 155)
            if not hit:
                (particles, hit) = attack_point(20, 155)
                if not hit:
                    (particles, hit) = attack_point(20, 30)
                    if not hit:
                        print " :( CANT FIND BOTTLE C "  
        
    (particles, hit) = mp.navigateToWaypoint(84, 30, particles, interface, motors)
    

    interface.terminate()
    
def nanas_attackplan():
    return ()
    


if __name__ == "__main__":
    main()