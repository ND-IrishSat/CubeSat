"""VectorNav VN-100 IMU driver (testbed IMU).

Purpose:  Implements gyro, magnetometer, and accelerometer interfaces for the
          VN-100 used on the BERTHA testbed.
Inputs:   ``Vn100Params`` (port, baud, sample rate).
Outputs:  ``Vn100Driver`` (``GyroInterface``, ``MagInterface``, ``AccelInterface``).
Status:   [MODIFIED]
Port from: legacy/HardwareInterface/vn100/vn100_interface.py
           (vendored library: ``vn100_library/``, install separately)

Changes from legacy:
    - Class instance instead of a module-level singleton ``VnSensor``.
    - Return ``Sample`` in SI: gyro deg/s -> rad/s, mag µT -> T
      (use ``geometry/units.py``).
    - No ``print``; no file logging (``print_data_to_file`` belongs in tools/).
    - Raw values only; mag calibration moves to ``estimation/mag_conditioning.py``.
	- NEW: for now, Talks the VN-100 ASCII protocol directly over pyserial; no ``vnpy``
      (vendored ``vn100_library/`` is no longer needed).

Protocol summary (VN-100 user manual):
    Command:  ``$<body>*<CS>\\r\\n`` where CS = XOR of the bytes of <body>, 2 hex digits.
    Read register:  ``VNRRG,<id>``    -> reply ``VNRRG,<id>,<v1>,<v2>,...``
    Write register: ``VNWRG,<id>,<v>`` -> reply echoes the write.
    Error:          ``VNERR,<code>``

Not the flight IMU; flight IMU driver added once the part is chosen (HW).
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np
import serial

from cloversat.datatypes.sample_types import Sample, SelfTestResult
from cloversat.datatypes.vector_types import Quat, Vec3
from cloversat.devices.interfaces.accel_interface import AccelInterface
from cloversat.devices.interfaces.gyro_interface import GyroInterface
from cloversat.devices.interfaces.mag_interface import MagInterface
from cloversat.geometry.units import gauss_to_t

if TYPE_CHECKING:
    from cloversat.config.device_params import Vn100Params

'''from the vn100 mannual: if the firmware rejects 54, use register 27 and offset 3 for the imu'''
_REG_MODEL = 1
_REG_ASYNC_TYPE = 6
_REG_QUAT = 9

_IMU_REG = 54
_IMU_OFFSET = 0

_REPLY_TIMEOUT_S = 0.5

'''indoor tesetbed elf-test plausibility bounds'''
_ACCEL_NORM_RANGE = (7.0, 12.5)    
_MAG_NORM_RANGE = (5e-6, 2e-4)     
_GYRO_NORM_MAX = 1.0 


'''organizing the real time output from the imu - the data is usually messy'''
def _checksum(body: str) -> str:
    """8-bit XOR of every byte between '$' and '*', as 2 uppercase hex digits."""
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return f"{cs:02X}"

def _frame(body: str) -> bytes:
    """'VNRRG,8' -> b'$VNRRG,8*4B\\r\\n'"""
    return f"${body}*{_checksum(body)}\r\n".encode("ascii")


def _unframe(line: bytes) -> Optional[str]:
    """Return the body of a received line if its checksum is valid, else None."""
    text = line.decode("ascii", errors="ignore").strip()
    if not text.startswith("$") or "*" not in text:
        return None
    body, _, cs = text[1:].partition("*")
    if cs[:2].upper() != _checksum(body):
        return None
    return body


def _invalid(t: float) -> Sample[Vec3]:
    return Sample(value=np.full(3, np.nan), t=t, valid=False)


class Vn100Driver(GyroInterface, MagInterface, AccelInterface):
    """VN-100 over serial."""

    def __init__(self, params: Vn100Params) -> None:
        self._params = params
        self._ser: Optional[serial.Serial] = None
        self.model: Optional[str] = None

    def init(self) -> None:
        """Connect and verify sensor connectivity."""
        if self._ser is not None and self._ser.is_open:
              return
        if self._params.port is None or self._params.baud is None:
              raise ValueError("set Vn100Params.port and Vn100Params.baud")
        self._ser = serial.Serial(self._params.port, self._params.baud, timeout=0.05)
        try:
            # Stop async streaming so polled replies are not buried in data lines.
            self._write_register(_REG_ASYNC_TYPE, 0)
            model = self._read_register(_REG_MODEL)[0]
            if not model.startswith("VN-100"):
                raise Vn100CommError(f"Unexpected model {model!r} on {self._params.port}")
            self.model = model
        except Exception:
            self.close()
            raise
    
    def close(self) -> None:
          '''call it when nothing is connected - disconnecting from sensor'''
          if self._ser is not None:
                try:
                      self._ser.close()
                finally:
                      self._ser = None
                
                  
		
   # def self_test(self) -> SelfTestResult:
    #    """Verify connectivity and plausible readings."""
#        raise NotImplementedError

    def read_gyro(self) -> Sample[Vec3]:
        """Return ``omega_body`` [rad/s]."""
        return self.read_all()[0]

    def read_mag(self) -> Sample[Vec3]:
        """Return ``B_body`` [T]."""
        return self.read_all()[1]

    def read_accel(self) -> Sample[Vec3]:
        """Return ``a_body`` [m/s^2]."""
        return self.read_all()[2]
    
    def read_all(self) -> tuple[Sample[Vec3], Sample[Vec3], Sample[Vec3]]:
              '''one poll of the sensor instead of three seperate reads when we are reading all of them at the same time'''
              self._require_open()
              try:
                    raw = self._read_register(_IMU_REG)
                    t = time.monotonic()
                    v = np.array([float(x) for x in raw[_IMU_OFFSET:_IMU_OFFSET + 9]], dtype=np.float64)
                    if v.size != 9 or not np.all(np.isfinite(v)):
                           raise Vn100CommError(f"Bad IMU reply: {raw}")
              except (Vn100CommError, TimeoutError, ValueError, serial.SerialException):
                    t = time.monotonic()
                    return _invalid(t), _invalid(t), _invalid(t)
                    
              mag = gauss_to_t(v[0:3])
              accel = v[3:6]
              gyro = v[6:9]
              return (Sample(gyro, t, True), Sample(mag, t, True), Sample(accel, t, True))
    
    def read_attitude(self) -> Sample[Quat]:
              self._require_open()
              try:
                    raw = self._read_register(_REG_QUAT)
                    t = time.monotonic()
                    x, y, z, w = (float(v) for v in raw[:4])
              except (Vn100CommError, TimeoutError, ValueError, serial.SerialException):
                    return Sample(np.full(4, np.nan), time.monotonic(), False)
              q = np.array([w, x, y, z])
              return Sample(q/np.linalg.norm(q), t, True)

	
    def self_test(self) -> SelfTestResult:
        """Verify connectivity and plausible readings. Assumes the sensor is still."""
        if self._ser is None or not self._ser.is_open:
            return SelfTestResult(False, 1, "not initialized")
        try:
            model = self._read_register(_REG_MODEL)[0]
        except (Vn100CommError, TimeoutError, serial.SerialException) as e:
            return SelfTestResult(False, 2, f"no response: {e}")

        gyro, mag, accel = self.read_all()
        if not (gyro.valid and mag.valid and accel.valid):
            return SelfTestResult(False, 3, "IMU measurement read failed")

        a = float(np.linalg.norm(accel.value))
        b = float(np.linalg.norm(mag.value))
        w = float(np.linalg.norm(gyro.value))
        if not _ACCEL_NORM_RANGE[0] <= a <= _ACCEL_NORM_RANGE[1]:
            return SelfTestResult(False, 4, f"|accel|={a:.2f} m/s^2 implausible")
        if not _MAG_NORM_RANGE[0] <= b <= _MAG_NORM_RANGE[1]:
            return SelfTestResult(False, 5, f"|B|={b * 1e6:.1f} uT implausible")
        if w > _GYRO_NORM_MAX:
            return SelfTestResult(False, 6, f"|omega|={w:.3f} rad/s, not stationary?")
        return SelfTestResult(True, 0, f"{model} ok")
	
	
	
	# low level protocol
    def _require_open(self) -> serial.Serial:
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Vn100Driver not initialized; call init() first")
        return self._ser

	# read & write to the register 
    def _read_register(self, reg: int) -> list[str]:
        return self._transact(f"VNRRG,{reg}", "VNRRG", reg)

    def _write_register(self, reg: int, *values: object) -> list[str]:
        body = f"VNWRG,{reg}," + ",".join(str(v) for v in values)
        return self._transact(body, "VNWRG", reg)

    def _transact(self, body: str, cmd: str, reg: int) -> list[str]:
        ser = self._require_open()
        ser.reset_input_buffer()              # drop stale data from earlier
        ser.write(_frame(body))

		
        buf = b""
        deadline = time.monotonic() + _REPLY_TIMEOUT_S
        while time.monotonic() < deadline:
            buf += ser.read(ser.in_waiting or 1)   # blocks <= serial timeout
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                reply = _unframe(line)
                if reply is None:
                    continue                  # partial or corrupt line
                fields = reply.split(",")
                if fields[0] == "VNERR":
                    raise Vn100CommError(f"{body!r} rejected: VNERR {fields[1:]}")
                if (fields[0] == cmd and len(fields) > 1
                        and fields[1].isdigit() and int(fields[1]) == reg):
                    return fields[2:]
        raise TimeoutError(f"No reply to {body!r} within {_REPLY_TIMEOUT_S}s")
              


	


	










# ---- LEGACY START: legacy/HardwareInterface/vn100/vn100_interface.py (reference only, not wired up) ----
# TODO(bug): legacy returns mag in µT and gyro in deg/s; port must return T and rad/s.
'''
vn100_interface.py
Authors: Andrew Gaylord

test script for connecting to and reading data from vn100 imu

Script for changing connectivity to VN100 IMU sensor, and reading and/or getting data from the sensor.
Data such as quaternion, magnetic, angle, acceleration, and temperature.


'''
# [scaffold: disabled] from vnpy import VnSensor
# [scaffold: disabled] import vnpy as Registers  # Create namespace alias for backwards compatibility
# [scaffold: disabled] s = VnSensor()

'''
Alternate connection route:

ez = EzAsyncData.connect('COM5', 115200)

# call the sensor object through the object and read the relevant data
ez.sensor.read_imu_measurements()

# this returns a compositeData object
# lots of helper functions exist to extract the data from this object
data = ez.current_data

print("angular: ", data.angular_rate)

print("quat: ", data.any_attitude.quat)

'''


def get_instance(): 
	'''
	@returns 
		The singleton instance of the VnSensor 
	'''
	return s
def connect(port = 'COM5'):
	'''
	Connects to the VnSensor version using specified COM port

	Prints "CONNECTED" upon succesful connection verification
	'''
	s.connect(port, 115200)  # Connect using vnpy API
	if s.verify_sensor_connectivity():
		print("CONNECTED")
		return True

def disconnect():
	'''
	Disconnects from the VnSensor version using specified COM port

	Prints "DICONNECTED" upon end
	'''
	s.disconnect()
	print("DISCONNECTED")

def read_quat():
	'''
	readquat(): reads current orientation from sensor in quaternion form

	@return 
		A 4 element list containing the quaternion values (w, x, y, z)
		Note: Quaternions do not have units
	'''
	quat = s.read_attitude_quaternion()
	# vnpy returns as Quaternion object, extract w, x, y, z
	return [quat.w, quat.x, quat.y, quat.z]

def read_mag():
	'''
	readmag(): Reads the current magnetic fields from the sensor as three values corresponding to x,y,z dimensions 

	@return 
		A 3 element list containing 3 float values in microtesla (x,y,z)
	'''
	mag = s.read_magnetic_measurements()
	return [mag.x, mag.y, mag.z]


def read_gyro():
	'''
	read_gyro(): reads angular velocity values from sensor

	@return 
		A returns 3 element list containing 3 float values in °/s (x,y,z)
	'''
	gyro = s.read_angular_rate_measurements()
	return [gyro.x, gyro.y, gyro.z]
	

def read_accel():
	'''
	read_accel(): reads acceleration value from sensor

	@return 
		A returns 3 element list containing 3 float values in m/s^2 (x,y,z)
	'''
	accel = s.read_acceleration_measurements()
	return [accel.x, accel.y, accel.z]

def read_all():
	'''
	read_all(): Gets magnetic field and angular velocity
	@return
		Returns a 2 element list containing all data from magnetic field and angular velocity
	'''
	mag = s.read_magnetic_measurements()
	gyro = s.read_angular_rate_measurements()
	return [[mag.x, mag.y, mag.z], [gyro.x, gyro.y, gyro.z]]

def get_mag_gyro_quat():
	'''
	get_mag_gyro_quat: Wraps mag gyro and quaternion data in a String

	@return
		A string comprised of three lists joined and seperated by commas (x,y,z) x (mag, gyro, quat)
	'''

	mag = [ str(a) for a in read_mag()]
	gyro = [ str(b) for b in read_gyro()]
	quat = [ str(c) for c in read_quat()]
	all_three = mag + gyro + quat # create list with all data
	all_three_string = ", ".join(all_three)
	return all_three_string

def print_data_to_file(count, file_name):
	'''
	print_data_to_file(): Updates a text file with data from Magnetometer, Gyroscope, and Accelerometer, seperating each data set on a new line
	
	@params
		count: Number of desried data sets
		file_name: Name of the file
	
	'''
	i = 0 # keep track of our iteration count
	f = open(file_name, "a+")
	while i < count:

		i += 1
		# save to text file in form of magnetometer (magnetic field), angular velocity (gyroscope), and acceleration (accelerometer)
		f.write(get_mag_gyro_quat()) # put mag, gyro, quat data into text file
		if (i < count): f.write("\n") # add newline to separate data sets

	f.close()
	#source = f'./{file_name}'
	#destination = './new_sensor_tests'
	#os.rename(source, destination)
	return

def print_mag_calibration():
	#TODO: Finish implementing this method
	'''
	Note: Unfinished
	
	print_mag_calibration
		A method that prints out the calculated calibration for both B and C
	'''
	mag_cal = s.read_calculated_magnetometer_calibration()
	print("b: ", mag_cal.b) # b and c are objects
	print("c: ", mag_cal.c)

#Returns temp from the sensor as 
def read_temp():
	'''
	read_temp(): reads temperature value from sensor
	@return 
		Returns a float with the temperature in °C
	'''
	return s.readRegister(Registers.IMU.ImuMeas).temperature

#Returns the pressure reading of the sensor as a float 
def read_pressure():
	'''
	read_pressure(): reads pressure value from sensor
	@return
		Returns a float with the pressure in Hectopascals (hPO)
	'''
	return s.readRegister(Registers.IMU.ImuMeas).pressure
# ---- LEGACY END ----
