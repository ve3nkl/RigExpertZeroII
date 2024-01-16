"""

  Communicate with RigExpert ZERO II board from RPi Pico using Micropython.
  (c) Copyright 2024, Nikolai Ozerov VE3NKL
  
  MIT License
  ----------------------------------------------------------------------
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ----------------------------------------------------------------------
  
"""

import machine
import time
import struct
from math import sqrt, log10

class RigExpertZeroII:
  
    """
    Supported board commands
    """
    CMD_GET_FW_VERSION = b'\xE5'
    CMD_GET_STATUS     = b'\x5A'
    CMD_SET_FQ_GET_RX  = b'\x6D'
    CMD_GET_SYSTEM_Z0  = b'\xC4'

    """
    Board status codes
    """
    ZEROII_STATUS_BUSY_USB  = 1 # The device is busy (USB interface)
    ZEROII_STATUS_BUSY_SPI  = 2 # The device is busy (SPI interface)
    ZEROII_STATUS_BUSY_I2C  = 3 # The device is busy (I2C interface)
    ZEROII_STATUS_BUSY_UART = 4 # The device is busy (UART interface)
    ZEROII_STATUS_IDLE      = 5 # The device is idle, ready for new tasks
    ZEROII_STATUS_READY     = 6 # Measurement results are ready
    ZEROII_STATUS_ERROR     = 7 # An error occurred while executing the task
    
    """
    Creating an object to control the board. All parameters have their defalts. Pay attention on what
    I2C interface number you are using in RPi Pico and what pins are used for this interface.
    After the object is created, the first method to be called is 'start_zeroii'.
    """
    def __init__(self, ZERO_I2C_ADDR=91, I2C_ID=0, SCL_PIN=17, SDA_PIN=16, RESET_PIN=18, I2C_FREQ=100000):
        self.ZERO_I2C_ADDR = ZERO_I2C_ADDR
        self.i2c = machine.I2C(id=I2C_ID, scl=machine.Pin(SCL_PIN), sda=machine.Pin(SDA_PIN), freq=I2C_FREQ)
        self.reset = machine.Pin(RESET_PIN, mode=machine.Pin.OUT)
        
        self.major_version = 0
        self.minor_version = 0
        self.hw_version    = 0
        self.serial_number = "?"
        
        self.MAX_FQ = 1000000000;
        
        self.Z0 = 0
        self.R = 0.0
        self.X = 0.0
        self.GAMMA = 1
        self.SWR = 200.0
        self.RL = 0
        self.ML = 0
        
        self.started = False
        
        self.resp_fw_version    = bytearray(7)
        self.resp_get_system_z0 = bytearray(4)
        self.resp_get_status    = bytearray(1)
        self.request_set_fq_get_rx = bytearray(5)
        self.resp_set_fq_get_rx = bytearray(8)


    """
    This method resets the board and makes it ready for performing RF measurements.
    """    
    def start_zeroii(self):
        # Reset the board
        time.sleep_ms(5)
        self.reset.value(0)
        time.sleep_ms(5)
        self.reset.value(1)
        time.sleep_ms(100)
        
        # Make sure the board is on the I2C bus
        devices = self.i2c.scan()
        self.started = False
        if devices:
            for d in devices:
                if d == self.ZERO_I2C_ADDR:
                    self.started = True
                    break;
        
        if self.started:
            if self.get_immediate_response(RigExpertZeroII.CMD_GET_FW_VERSION, self.resp_fw_version):
                self.major_version = self.resp_fw_version[0] + 0
                self.minor_version = self.resp_fw_version[1] + 0
                self.hw_version    = self.resp_fw_version[2] + 0
                self.serial_number = self.byte2hex(self.resp_fw_version[3]) + \
                                     self.byte2hex(self.resp_fw_version[4]) + \
                                     self.byte2hex(self.resp_fw_version[5]) + \
                                     self.byte2hex(self.resp_fw_version[6])
                if self.get_immediate_response(RigExpertZeroII.CMD_GET_SYSTEM_Z0, self.resp_get_system_z0):
                    self.Z0 = struct.unpack('i', self.resp_get_system_z0)[0]
                    return True
                else:
                    return False
            
        return False
    

    """
    This is an internal method for performing requests that have imemdiate response. 
    """
    def get_immediate_response(self, command, resp_array):
        self.i2c.writeto(self.ZERO_I2C_ADDR, command)
        resp_OK = False
        tries = 0

        while True:
            time.sleep_us(100)
            try:
                self.i2c.readfrom_into(self.ZERO_I2C_ADDR, resp_array)
                resp_OK = True
                break
            except:
                tries += 1
            if tries > 5000:
                break
        return resp_OK
    
    """
    Helper method to convert byte value into hexadecimal 2 digit number.
    """
    def byte2hex(self, b):
        if b < 16:
            return "0" + hex(b)[2:]
        else:
            return hex(b)[2:]
    
    """
    After 'start_zeroii' is called, this method will return a major version number
    of the firmware installed on the board.
    """
    def get_major_version(self):
        return self.major_version
    
    """
    After 'start_zeroii' is called, this method will return a minor version number
    of the firmware installed on the board.
    """
    def get_minor_version(self):
        return self.minor_version
    
    """
    After 'start_zeroii' is called, this method will return the board's hardware revision.
    """
    def get_hw_revision(self):
        return self.hw_version

    """
    After 'start_zeroii' is called, this method will return the board's serial number.
    """    
    def get_serial_number(self):
        return self.serial_number
        
    """
    This is the 1-st method of 3 to asynchronously perform RF measurements. The 
    'frequency' parametr is specified in Hz.
    """    
    def measure_start(self, frequency):
        if self.started:
            if frequency > self.MAX_FQ:
                frequency = self.MAX_FQ
            self.request_set_fq_get_rx[0] = RigExpertZeroII.CMD_SET_FQ_GET_RX[0]
            self.request_set_fq_get_rx[1:] = frequency.to_bytes(4, "little")
            self.i2c.writeto(self.ZERO_I2C_ADDR, self.request_set_fq_get_rx)
            return True
        else:
            return False
       
    """
    This is the 2-nd method of 3 to asynchronously perform RF measurements. It should be
    called periodically until it returns True. It will indicate that the results are ready
    to be obtained.
    """
    def measure_is_result_ready(self):
        status = self.get_status()
        if status == RigExpertZeroII.ZEROII_STATUS_READY:
          return True
        else: 
          return False
          
    """
    This is the 3-rd method of 3 to asynchronously perform RF measurements. When 
    'measure_is_result_ready' returned True, this method can be called. It returns 
    True if no errors occurs, meaning that the result was obtained. To actually access
    the obtained result one of the 'get_obtained_xxx' methods should be called.
    """
    def measure_obtain_result(self):
        try:
            self.i2c.readfrom_into(self.ZERO_I2C_ADDR, self.resp_set_fq_get_rx)
            self.R = struct.unpack('f', self.resp_set_fq_get_rx[0:4])[0]
            self.X = struct.unpack('f', self.resp_set_fq_get_rx[4:8])[0]
            self.GAMMA, self.SWR, self.RL, self.ML = self.compute_values()
            return True
        except:
            return False
    
    """
    This method is an aletrantive to 3 methods above. It will measuer and obtain a result
    synchronously. It means that it may lock for some time (typically milliseconds). If it
    returns True it means that the result was obtained. To actually access the obtained result
    one of the 'get_obtained_xxx' methods should be called.
    """        
    def measure(self, frequency):
        if self.started:
            self.measure_start(frequency)
            while not self.measure_is_result_ready():
              time.sleep_us(500)
            if self.measure_obtain_result():
              return True
            else:
              return False
        else:
          return False
          
    
    """
    This method returns system impedance (Z0) in thousandth of Ohm. I.e., for 50
    Ohm the return value would be 50000.
    """
    def get_system_z0(self):
        return self.Z0
    
    """
    This method returns the current status of the board.
    """
    def get_status(self):
        if self.get_immediate_response(RigExpertZeroII.CMD_GET_STATUS, self.resp_get_status):
            return self.resp_get_status[0]
        else:
            return RigExpertZeroII.ZEROII_STATUS_ERROR
    
    """
    This is an internal method to compute various RF values. 
    """    
    def compute_values(self):
        Z0 = self.Z0 / 1000.0
        R  = self.R
        X  = self.X
        if R <= 0:    # Replace with a small value
            R = 0.001
        denominator = (R + Z0) * (R + Z0) + X * X
        if denominator > 0:
            gamma = sqrt(((R - Z0) * (R - Z0) + X * X) / denominator)
            if gamma < 1.0:
                swr = (1 + gamma) / (1 - gamma)
                if swr > 200 or gamma > 0.99:
                    swr = 200.0
                elif swr < 1:
                    swr = 1.0
            else:
                swr = 200.0
                gamma = (swr - 1)/(swr + 1)
        else:
            swr = 200
            gamma = (swr - 1)/(swr + 1)
            
        w = (swr-1)/(swr+1)    
        mismatch_loss = -10 * log10(1 - w * w)
        return_loss   = -20 * log10(gamma)
        return (gamma, swr, return_loss, mismatch_loss)
    
    """
    This method can be used to access the measured 'R' value (the real part of the 
    measured impedance) in Ohm.
    """
    def get_obtained_r(self):
        return self.R
    
    """
    This method can be used to access the measured 'X' value (the imaginary part of the 
    measured impedance) in Ohm.
    """
    def get_obtained_x(self):
        return self.X
    
    """
    This method can be used to access the measured 'SWR' value.
    """
    def get_obtained_swr(self):
        return self.SWR

    """
    This method can be used to access the measured 'Return Loss' value in dB.
    """
    def get_obtained_return_loss(self):
        return self.RL

    """
    This method can be used to access the measured 'Mismatch Loss' value in dB.
    """
    def get_obtained_mismatch_loss(self):
        return self.ML
