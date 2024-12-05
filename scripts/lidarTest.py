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
            sleep(0.1)
            t0 = time()
            self.car.log_t_vec = []
            self.car.lidar_vec = []
            while time() < t0 + dt:
                self.car.ready.wait()
                self.car.ready.clear()
                self.car.throttle = throttle
                self.car.steering = sin(2*pi/self.T*time()) * radians(26.1)
                self.print_info('command:',self.car.throttle,self.car.steering)

            self.t_vec = self.car.log_t_vec
            self.lidar_vec = self.car.lidar_vec
        except KeyboardInterrupt:
            pass
        finally:
            self.print_info("waiting to quit")
            self.car.quit()

    def final(self):
        lidar = np.array(self.lidar_vec)
        t_vec = np.array(self.t_vec)
        t_vec -= t_vec[0]
        fig, ax = plt.subplots(4,1)
        ax[0].plot(t_vec, lidar[:,0],'-*')
        ax[1].plot(t_vec, lidar[:,1],'-*')
        ax[2].plot(t_vec, lidar[:,2],'-*')
        ax[3].plot(t_vec, lidar[:,3],'-*')
        plt.show()
        pass

if __name__ == '__main__':
    T = 0.7
    main = SinSteeringTest(T) 
    main.main()
    main.final()

