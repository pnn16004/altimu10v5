import rclpy
from rclpy.node import Node
from time import sleep

from std_msgs.msg import Float32MultiArray

from submodules.lsm6ds33 import LSM6DS33
from submodules.lis3mdl import LIS3MDL
#from submodules.lps25h import LPS25H

from ahrs.filters import Madgwick
from ahrs.filters import Mahony
from ahrs.common import Quaternion

import math
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.lsm6ds33 = LSM6DS33()
        self.gyroAccelEnabled = False
        self.lis3mdl = LIS3MDL()
        self.magnetometerEnabled = False
        #self.lps25h = LPS25H()
        #self.barometerEnabled = False
        
        self.enable(barometer=False)
        self.lis3mdl.calibrate_magnet(iterations=1000)
        self.lsm6ds33.calibrate_gyro(iterations=500)

        freq = 50

        self.publisher_ = self.create_publisher(Float32MultiArray, '/position/test', freq)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.orientation = Madgwick(frequency=freq)
        #self.orientation = Mahony(frequency=freq)
        self.Q = np.array([1., 0., 0., 0.])
        self.oldQ = self.Q

        self.count = 0
        self.MAF = 4;
        self.bufferRPY = np.empty((3,0), float)
        
        self.RPY_cal = [0, 0, 0]
        self.is_RPY_calibrated = False
        self.calibrate_RPY(iterations=200)
        self.oldRPY = np.array([0, 0, 0])
        self.sumRPY = np.array([0, 0, 0])
    
    def __del__(self):
        del(self.lsm6ds33)
        del(self.lis3mdl)
        #del(self.lps25h)
    
    def enable(self, gyroAccel=True, magnetometer=True, barometer=True):
        """ Enable the given devices. """
        if gyroAccel:
            self.lsm6ds33.enable()
            self.gyroAccelEnabled = True
        if magnetometer:
            self.lis3mdl.enable()
            self.magnetometerEnabled = True
        #if barometer:
            #self.lps25h.enable()
            #self.barometerEnabled = True
    
    def timer_callback(self):
        msg = Float32MultiArray()
        
        #accAngles = self.lsm6ds33.get_accel_angles()
        #compAngles = self.lsm6ds33.get_complementary_angles()
        #self.get_logger().info('compAngles: "%s and %s"' % (compAngles[0], compAngles[1]))

        RPY = self.get_RPY()

        self.bufferRPY = np.c_[RPY, self.bufferRPY]
        if (self.count >= self.MAF):
            self.bufferRPY = np.delete(self.bufferRPY, (self.MAF-1), axis=1)
            #self.bufferRPY = self.bufferRPY[:, :self.MAF]

        if ((self.count % 2 == 0) & (self.count >= self.MAF)):
            RPY = np.sum(self.bufferRPY, 1) / self.MAF
            msg.data = [float(i * 180/math.pi) for i in RPY]
            self.publisher_.publish(msg)

            self.sumRPY = np.add(self.sumRPY, np.subtract(msg.data, self.oldRPY))
            #print(self.sumRPY)
            self.oldRPY = msg.data

        self.count = self.count + 1

    def get_RPY(self):

        #accAngles = self.lsm6ds33.get_accel_raw()
        #gyroAngles = self.lsm6ds33.get_gyroscope_raw()
        magAngles = self.lis3mdl.get_magnetometer_raw()

        #accGravity = get_accel_g_forces()
        accAngles = self.lsm6ds33.get_accel_linear_velocity()
        gyrAngles = self.lsm6ds33.get_gyro_angular_velocity()

        self.get_logger().info('accAngles: "%s and %s and %s"' % (accAngles[0], accAngles[1], accAngles[2]))
        self.get_logger().info('gyroAngles: "%s and %s and %s"' % (gyrAngles[0], gyrAngles[1], gyrAngles[2]))
        self.get_logger().info('magAngles: "%s and %s and %s"' % (magAngles[0], magAngles[1], magAngles[2]))

        self.Q = self.orientation.updateMARG(self.oldQ, gyr=gyrAngles, acc=accAngles, mag=magAngles)
        self.oldQ = self.Q
        RPY = Quaternion(self.Q).to_angles()

        if self.is_RPY_calibrated:
            RPY -= self.RPY_cal

        return RPY

    
    def calibrate_RPY(self, iterations=2000):
        """Calibrate RPY """
        print('Calibrating RPY...')

        for i in range(iterations):
            RPY_raw = self.get_RPY()
            self.RPY_cal += RPY_raw

            sleep(0.004)

        self.RPY_cal /= iterations
        self.RPY_cal[:] = [x / iterations for x in self.RPY_cal] 

        print('Calibration Done')
        self.is_RPY_calibrated = True

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
