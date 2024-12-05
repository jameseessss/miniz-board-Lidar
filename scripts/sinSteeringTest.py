# send sinusoidal steering command
from math import pi,radians,degrees,sin
from common import *
from Offboard import Offboard
from time import time,sleep
import matplotlib.pyplot as plt

class SinSteeringTest(PrintObject):
    def __init__(self,T):
        self.car = Offboard("192.168.10.102",2390)
        sleep(0.1)
        self.T = T
        return

    def main(self):
        try:
            throttle = 0.0
            dt = 1.5
            self.car.setParam(1.5,0,0.05) 
            sleep(0.1)
            t0 = time()
            while time() < t0 + dt:
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = throttle
                self.car.steering = sin(2*pi/self.T*time()) * radians(26.1)
                self.print_info('command:',self.car.throttle,self.car.steering)

            self.old_t_vec = self.car.log_t_vec
            self.old_steering_requested_vec = self.car.steering_requested_vec
            self.old_steering_measured_vec = self.car.steering_measured_vec
            self.car.log_t_vec = []
            self.car.steering_requested_vec = []
            self.car.steering_measured_vec = []

            self.car.setParam(1.5,0,0.05) 
            sleep(0.1)
            t0 = time()
            self.T = 1.0

            while time() < t0 + dt:
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = throttle
                self.car.steering = sin(2*pi/self.T*time()) * radians(26.1)
                self.print_info('command:',self.car.throttle,self.car.steering)

            self.new_t_vec = self.car.log_t_vec
            self.new_steering_requested_vec = self.car.steering_requested_vec
            self.new_steering_measured_vec = self.car.steering_measured_vec
        except KeyboardInterrupt:
            pass
        finally:
            self.print_info("waiting to quit")
            self.car.quit()

    def final(self):
        fig, ax = plt.subplots(2,1)
        ax[0].plot(np.array(self.old_t_vec) - self.old_t_vec[0], self.old_steering_requested_vec)
        ax[0].plot(np.array(self.old_t_vec) - self.old_t_vec[0], self.old_steering_measured_vec,'*-')
        ax[1].plot(np.array(self.new_t_vec) - self.new_t_vec[0], self.new_steering_requested_vec)
        ax[1].plot(np.array(self.new_t_vec) - self.new_t_vec[0], self.new_steering_measured_vec,'*-')
        plt.show()
        pass

if __name__ == '__main__':
    T = 0.7
    main = SinSteeringTest(T) 
    main.main()
    main.final()

