# -*- coding: utf-8 -*-

"""Python library module for LSM6DS33 accelerometer and gyroscope.
This module for the Raspberry Pi computer helps interface the LSM6DS33
accelerometer and gyro.The library makes it easy to read
the raw accelerometer and gyro data through I²C interface and it also provides
methods for getting angular velocity and g forces.

The datasheet for the LSM6DS33 is available at
[https://www.pololu.com/file/download/LSM6DS33.pdf?file_id=0J1087]
"""

import math
from .i2c import I2C
from time import sleep
from .constants import *


class LSM6DS33(I2C):
    """ Set up and access LSM6DS33 accelerometer and gyroscope."""
    # Output registers used by the gyroscope
    gyro_registers = [
        LSM6DS33_OUTX_L_G,  # low byte of X value
        LSM6DS33_OUTX_H_G,  # high byte of X value
        LSM6DS33_OUTY_L_G,  # low byte of Y value
        LSM6DS33_OUTY_H_G,  # high byte of Y value
        LSM6DS33_OUTZ_L_G,  # low byte of Z value
        LSM6DS33_OUTZ_H_G,  # high byte of Z value
    ]

    # Output registers used by the accelerometer
    accel_registers = [
        LSM6DS33_OUTX_L_XL,  # low byte of X value
        LSM6DS33_OUTX_H_XL,  # high byte of X value
        LSM6DS33_OUTY_L_XL,  # low byte of Y value
        LSM6DS33_OUTY_H_XL,  # high byte of Y value
        LSM6DS33_OUTZ_L_XL,  # low byte of Z value
        LSM6DS33_OUTZ_H_XL,  # high byte of Z value
    ]

    def __init__(self, bus_id=1):
        """ Set up I2C connection and initialize some flags and values."""
        super(LSM6DS33, self).__init__(bus_id)
        self.is_accel_enabled = False
        self.is_gyro_enabled = False

        self.is_gyro_calibrated = False
        self.gyro_cal = [0, 0, 0]

        self.is_accel_calibrated = False
        self.accel_angle_cal = [0, 0]

    def __del__(self):
        """ Clean up."""
        try:
            # Power down accelerometer and gyro
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x00)
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x00)
            super(LSM6DS33, self).__del__()
            print('Destroying')
        except:
            pass

    def enable(self, accelerometer=True, gyroscope=True):
        """ Enable and set up the given sensors in the IMU."""
        if accelerometer:
            # 1.66 kHz (high performance) / +/- 4g
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x58)
            self.is_accel_enabled = True
        if gyroscope:
            # 208 Hz (high performance) / 1000 dps
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x58)
            self.is_gyro_enabled = True

    def calibrate_gyro(self, iterations=2000):
        """ Calibrate the gyro's raw values."""
        print('Calibrating Gyroscope...')

        for i in range(iterations):
            gyro_raw = self.get_gyro_raw()
            self.gyro_cal += gyro_raw

            sleep(0.004)

        self.gyro_cal[:] = [x / iterations for x in self.gyro_cal]

        print('Calibration Done')
        self.is_gyro_calibrated = True

    def calibrate_accel_angle(self, iterations=2000):
        """ Calibrate the acceleration angles (orientation) values."""
        print('Calibrating Accelerometer angles...')

        for i in range(iterations):
            accel_angles = self.get_accel_angles()                         
            self.accel_angle_cal += accel_angles

        sleep(0.004)                                                          
        
        self.accel_angle_cal /= iterations       
        self.accel_angle_cal[:] = [x / iterations for x in self.accel_angle_cal] 
            
        print('Calibration Done')
        self.is_accel_calibrated = True

    def get_gyro_raw(self):
        """ Return a 3D vector of raw gyro data."""

        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        sensor_data = self.read_3d_sensor(LSM6DS33_ADDR, self.gyro_registers)

        # Return the vector
        if self.is_gyro_calibrated:
            calibrated_gyro_data = sensor_data
            calibrated_gyro_data[0] -= self.gyro_cal[0]
            calibrated_gyro_data[1] -= self.gyro_cal[1]
            calibrated_gyro_data[2] -= self.gyro_cal[2]
            return calibrated_gyro_data
        else:
            return sensor_data

    def get_gyro_angular_velocity(self):
        """ Return a 3D vector of the angular velocity measured by the gyro
            in radians/second.
        """
        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        if not self.is_gyro_calibrated:
            raise(Exception('Gyroscope is not calibrated!'))

        gyro_data = self.get_gyro_raw()

        gyro_data[0] *= GYRO_GAIN / 1000 * math.pi/180

        return gyro_data

    def get_accel_raw(self):
        """ Return a 3D vector of raw accelerometer data."""

        if not self.is_accel_enabled:
            raise(Exception('Accelerometer is not enabled!'))

        return self.read_3d_sensor(LSM6DS33_ADDR, self.accel_registers)

    def get_accel_linear_velocity(self):
        """ Return a 3D vector of the g forces measured by the accelerometer
            removed, returning the linear velocity in m/s²
        """
        if not self.is_accel_enabled:                                                           
            raise(Exception('Accelerometer is not enabled!')) 

        [x_val, y_val, z_val] = self.get_accel_raw()

        x_val -= (x_val * ACCEL_CONVERSION_FACTOR) / 1000 * 8192
        y_val -= (y_val * ACCEL_CONVERSION_FACTOR) / 1000 * 8192
        z_val -= (z_val * ACCEL_CONVERSION_FACTOR) / 1000 * 8192

        return [x_val, y_val, z_val]

    def get_accel_angles(self, round_digits=0):
        """ Return a 2D vector of roll and pitch angles,
            based on accelerometer g forces
        """

        # Get raw accelerometer g forces
        [acc_xg_force, acc_yg_force, acc_zg_force] = self.get_accel_g_forces()

        # Calculate angles
        xz_dist = self._get_dist(acc_xg_force, acc_zg_force)
        yz_dist = self._get_dist(acc_yg_force, acc_zg_force)
        accel_roll_angle = math.degrees(math.atan2(acc_yg_force, xz_dist))
        accel_pitch_angle = -math.degrees(math.atan2(acc_xg_force, yz_dist))

        if self.is_accel_calibrated:
            accel_roll_angle -= self.accel_angle_cal[0]
            accel_pitch_angle -= self.accel_angle_cal[1]
            if round_digits != 0:
                return [round(accel_roll_angle, round_digits), round(accel_pitch_angle, round_digits)]
            else:
                return [accel_roll_angle, accel_pitch_angle]
        else:
            return [accel_roll_angle, accel_pitch_angle]

    def get_complementary_angles(self, delta_t=0.05):
        """ Calculate combined angles of accelerometer and gyroscope
        using a complementary filter.
        """
        if (not self.is_gyro_enabled) & (not self.is_accel_enabled):
            raise(Exception('Gyroscope and accelerometer are not enabled!'))

        self.complementary_angles = [0, 0]
        complementary_filter_constant = 0.98

        accel_angles = self.get_accel_angles()
        gyro_angular_velocity = self.get_gyro_angular_velocity()

        self.complementary_angles[0] = complementary_filter_constant                   \
            * (self.complementary_angles[0] + (gyro_angular_velocity[0] * delta_t))    \
            + (1 - complementary_filter_constant)                                      \
            * accel_angles[0]
        self.complementary_angles[1] = complementary_filter_constant                   \
            * (self.complementary_angles[1] + (gyro_angular_velocity[1] * delta_t))    \
            + (1 - complementary_filter_constant)                                      \
            * accel_angles[1]

        return self.complementary_angles

    def _get_dist(self, a, b):
        return math.sqrt((a * a) + (b * b))
