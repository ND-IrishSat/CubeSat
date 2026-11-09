import csv

from typing import TYPE_CHECKING

import os

import numpy

from legacy.params import MAX_PWM
import pigpio

"""
Reaction Wheel Simulation: 
1. Map torque body request to torque request per wheel
2. Map that voltage to pwm
3. Send that pwm to pin 

pwm (pulse width modulation): like antilock brakes (on and off max 50% of the time)

sources: https://docs.google.com/document/d/1ZfniI7BwBoRzMWSKxQ3m8HrUjTOSw5HzS2pyHeFh7E8/edit?usp=sharing 

paper notes: https://drive.google.com/file/d/1A0r9c32eGkZr2fDVYfKHC6ajhtUSqVIR/view?usp=sharing 
"""

class ReactionWheelSims:
    def __init__(self, W, J):
        self.W = W  #Ask mechanical team for spin axes of reaction wheels. 
                    #W  [r1, ..., rn] where ri is the unit vector of the ith
                    #reaction wheel spin axis (AVS) - from "Autonomous Vehicle Simulation (AVS) Laboratory, University of Colorado"

        self.J = J #RW spin intertia about spin axis
                    #Calculated from CAD design

        #NOTE: I just put in a random value for each of these to test it, find and import from parameter files later
        MAX_VOLTAGE = 5 #max voltage for reaction wheels, estimated with claude
        MIN_VOLTAGE = 0.1 #min voltage for reaction wheels, defined as voltage below which motor does not apply torque 
                        #estimated with claude
        MAX_TORQUE = 5 #max torque reaction wheels, estimated with claude
        MAX_PWM = 0.95 #pulse width modulation, estimated with claude


    def torque_to_pwm(self, L: float): #L is torque body request for entire cubesat - should be result from magnetorquer
        torque_wheels = []
        torque_wheels = numpy.linalg.pinv(W) @ L
        #eq 7 from https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10850201&tag=1 
        # previous is pseudoinverse, this is manual calculation: torque_wheels = numpy.transpose(W)@numpy.linalg.inv((W@numpy.transpose(W)))@L
        return torque_wheels

    def torque_closed_loop_control(self, torque_wheels, K): #K is control constant - can calculate or guess and check stability
        #torque = J*angular_acceleration (J is reaction wheel spin inertia about spin axis)
        #eq 3 from https://hanspeterschaub.info/bskOlderDocs/bsk_2_7_35/_downloads/6e9ffc059210810ed0b61270693500ae/Basilisk-rwMotorVoltage-20170113.pdf 
        #J is 3x3 matrix with values for 1,2,3 on diagonal, ang_accel is 3x1 matrix
        ang_vel = []
        ang_vel = numpy.linalg.inv(J) @ torque_wheels #divide torque by J from previous equation to digitally calculate angular velocity

        #get actual angular acceleration from motors? is that possible
        ang_accel = []
        ang_accel = [3, 3, 3] #Mechanical team said we can get velocity (comes through as frequency value)

        torque_actual = []
        torque_actual = numpy.linalg.diagonal(J)@ang_accel

        #Closed loop calculation to update torque applied to wheels based on actual speed
        torque_applied = []
        torque_applied = torque_wheels - K*(torque_actual - torque_wheels) 
        #eq 6 from https://hanspeterschaub.info/bskOlderDocs/bsk_2_7_35/_downloads/6e9ffc059210810ed0b61270693500ae/Basilisk-rwMotorVoltage-20170113.pdf 
        return torque_applied

    def torque_applied_to_voltage(self, torque_applied, MAX_VOLTAGE, MIN_VOLTAGE, MAX_TORQUE):
        voltage_int = []
        voltage = []

        voltage_int = (MAX_VOLTAGE - MIN_VOLTAGE)/MAX_TORQUE * torque_applied
        voltage = voltage_int + MIN_VOLTAGE*numpy.sign(voltage_int)
        #equation 2 from https://hanspeterschaub.info/bskOlderDocs/bsk_2_7_35/_downloads/6e9ffc059210810ed0b61270693500ae/Basilisk-rwMotorVoltage-20170113.pdf

    def voltage_to_pwm_to_pin(pi, pin, torque_applied):
        #FROM PID_controller.py

        pwm = (torque_applied / MAX_TORQUE) * MAX_PWM
        
        #Convert to integer values for actual PWM signals
        pwm = numpy.np.array([int(p) for p in pwm])
        
        # Ensure PWM is within bounds of motor constraints
        pwm = numpy.np.clip(pwm, -MAX_PWM * 1.0, MAX_PWM * 1.0)
                
        pi.set_mode(pin, pwm)
