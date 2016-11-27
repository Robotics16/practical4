import motor_particles as mp
import brickpi
import motor_util as mu
from motor_util import motors

def main():
    interface = brickpi.Interface()
    interface.initialize()

    mu.setUpInterface(interface)
    hit = False
    back = -10
    
    # Setup initial state of particles
    (hit, _) = mu.forward(40, interface, motors)
    if hit:
        mu.forward(back, interface, motors)
    mu.rotate(3.14, interface, motors)
    (hit, _) = mu.forward(40, interface, motors)
    if hit:
        mu.forward(back, interface, motors)
    mu.rotate(-3.14, interface, motors)
    

    interface.terminate()


if __name__ == "__main__":
    main()