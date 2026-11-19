from pathlib import Path
from cloversat.devices.sim.gyro_sim import GyroSim

gyro = GyroSim(Path("cloversat/devices/sim/test.csv"))

print(gyro.self_test())  # Expected: failed, not initialized

gyro.init()

print(gyro.self_test())  # Expected: passed
print(gyro.read_gyro())  # Read first CSV sample
print(gyro.read_gyro())  # Read second CSV sample