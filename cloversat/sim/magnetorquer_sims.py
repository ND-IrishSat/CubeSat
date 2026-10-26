

import csv

from typing import TYPE_CHECKING

import os

"""
Magnetorquer: interact with Earth's magnetic field to control the orientation of the satellite

Goal of function: converts a desired magnetic moment to the voltage needed to generate that moment
        NOTE: all in body frame, which is why area can be treated as [0, 0 , A]
Formulas: 
m = epsilon * n * I * A
    n = turns
    I = current (A)
    A = coil area (m^2)
    epsilon = check back in parameters/constructor

I = m / (epsilon * n * A)

Ohm's Law: 
V = I * R
    R = resistance of coil (ohms)
    V = voltage (V)

V = m * R / (epsilon * n * A)

Coil resistance: 
R = rho * L / A_wire
    pho  = resistivity of wire 
    L = length of wire (m)
    A_wire = cross-sectional area of wire (m^2)

!! Important - Final formula:
Voltage for moment = m * rho * L / (epsilon * n * A * A_wire)    
"""

class MagnetorquerSims:

    def __init__(self, n: int, area: float, k: float, epsilon: float):
        """
        Parameters:
            n (int): number of coil turns for each magnetorquer (num of layers * num of turns per layer)
            area (float): area of magnetorquer (m^2)
            k (float): detumbling constant gain for each magnetorquer (controls need to calculate later)
            epsilon (float): added multiplication term for ferromagnetic magnetorquers, 1 for air torquers (controls need to calculate later)
        """

        self.n = n
        self.area = area
        self.k = k
        # accounts for the magnetic permeability and the geometry of the core. 1 for air-core magnetorquers
        self.epsilon = epsilon

    def voltages_from_file(self, file_path: str):
        voltages = []
        print("Enter L:")
        L = float(input())
        print("Enter A_wire:")
        A_wire = float(input())
        print("Vary: 1. epsilon 2. m 3. rho")
        varied_parameter = int(input())
        if varied_parameter == 1:
            print("Enter constant m:")
            m = input()
            print("Enter constant rho:")
            rho = input()
        elif varied_parameter == 2: 
            print("Enter constant epsilon:")
            epsilon = input()
            print("Enter constant rho:")
            rho = input()
        elif varied_parameter == 3: 
            print("Enter constant epsilon:")
            epsilon = input()
            print("Enter constant m:")
            m = input()
        
        with open(file_path, "r") as f:
            data = csv.reader(f)
            for row in data:
                if varied_parameter == 1:
                    #Call the moment_to_voltage method using constant m and rho and the CSV file of varied epsilon
                    voltages.append(self.moment_to_voltage(float(m), float(rho), L, float(row[0]), self.n, self.area, A_wire))
                elif varied_parameter == 2: 
                    #Call moment to voltage method using constant epsilon and rho and the CSV file of varied m
                    voltages.append(self.moment_to_voltage(float(row[1]), float(rho), L, float(epsilon), self.n, self.area, A_wire))
                else: 
                    #Call moment to voltage method using constant epsilon and m and vary rho with CSV file
                    voltages.append(self.moment_to_voltage(float(m), float(row[2]), L, float(epsilon), self.n, self.area, A_wire))
        return voltages
    
    def moment_to_voltage(self, m: float, rho: float, L: float, epsilon: float, n: float, A: float, A_wire: float):
        return m * rho * L / (epsilon * n * A * A_wire)

    def magnetorquer_calculation_steps(self, m: float, rho: float, L: float, epsilon: float, n: float, A: float, A_wire: float):
        I = m / (epsilon * n * A)
        R = rho * L / A_wire
        V = I * R
        return "Current (I):" + str(I) + ", Resistance (R):" + str(R) + ", Voltage (V):" + str(V)


if __name__ == '__main__':
    sim = MagnetorquerSims(n=100, area=0.01, k=1.0, epsilon=1.0)
    voltages = sim.voltages_from_file(os.path.join(os.path.dirname(__file__), "magnetorquer_sim_test.csv"))
    print(voltages)
    
