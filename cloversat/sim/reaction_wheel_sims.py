import csv

from typing import TYPE_CHECKING

import os

import numpy

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
        self.W = W  #Ask mechanical team for spin axes of reaction wheels. W  [r1, ..., rn] where ri is the unit vector of the ith
                    #reaction wheel spin axis (AVS) - from "Autonomous Vehicle Simulation (AVS) Laboratory, University of Colorado"

        self.J = J #Ask at meeting if this is calculated or found from CAD or something
                #RW spin intertia about spin axis

        #NOTE: I just put in a random value for each of these to test it, they should be imported from some params file I think
        MAX_VOLTAGE = 5 #max voltage for reaction wheels, estimated with claude
        MIN_VOLTAGE = 0.1 #min voltage for reaction wheels, defined as voltage below which motor does not apply torque 
                        #estimated with claude
        MAX_TORQUE = 5 #max torque reaction wheels, estimated with claude
        MAX_PWM = 0.95 #pulse width modulation, estimated with claude


    def torque_to_pwm(self, L: float): #L is torque body request for entire cubesat - should be result from magnetorquer
        torque_wheels = []
        torque_wheels = numpy.linalg.pinv(W) @ L
        # previous is pseudoinverse, this is manual calculation: torque_wheels = numpy.transpose(W)@numpy.linalg.inv((W@numpy.transpose(W)))@L
        """
        Note: @ is for matrix multiplication instead of *
        Note: can also use numpy.linalg.pinv(W) @ L instead but keep formula for learning
        Eq 5 from "Autonomous Vehicle Simulation (AVS) Laboratory, University of Colorado, Basilisk Technical Memorandum
        Document ID: Basilisk-rwMotorVoltage" (link to paper in ella document)
        """
        return torque_wheels

    def torque_closed_loop_control(self, torque_wheels, K): #K is control constant - can calculate or guess and check stability I think
        #torque = J*angular_acceleration (J is reaction wheel spin inertia about spin axis)
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
        #control equation from "Autonomous Vehicle Simulation (AVS) Laboratory, University of Colorado, Basilisk Technical Memorandum, RWMOTORVOLTAGE
    
        return torque_applied

    def torque_applied_to_voltage(self, torque_applied, VMAX, VMIN, TORQUEMAX):
        voltage_int = []
        voltage = []

        voltage_int = (MAX_VOLTAGE - MIN_VOLTAGE)/MAX_TORQUE * torque_applied
        voltage = voltage_int + MIN_VOLTAGE*numpy.sign(voltage_int)
        #equation 2 from "Autonomous Vehicle Simulation (AVS) Laboratory, University of Colorado, Basilisk Technical Memorandum, RWMOTORVOLTAGE
        return voltage

    def voltage_to_pwm_to_pin(self, voltages): 
        #referenced wheel_allocation.py, is this right?
        pwm = []
        pwm = (voltages / MAX_VOLTAGE) * MAX_PWM
