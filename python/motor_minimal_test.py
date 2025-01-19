#!/usr/bin/env python3

import time
import struct
import smbus

# 1) I2C bus number (on Pi 4, typically 1; on Pi 5, you may need to try 4, 6, etc.)
I2C_BUS = 1

# 2) Motor driver I2C address
MOTOR_ADDR = 0x34

# 3) Register addresses (from your example)
ADC_BAT_ADDR               = 0x00  # Battery voltage (2 bytes)
MOTOR_TYPE_ADDR            = 0x14  # Motor type
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_PWM_ADDR       = 0x1F  # Open-loop PWM control register
MOTOR_FIXED_SPEED_ADDR     = 0x33  # Closed-loop ?fixed speed? register
MOTOR_ENCODER_TOTAL_ADDR   = 0x3C  # Accumulated encoder counts (16 bytes, 4x int32)

# 4) Possible motor types
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT              = 1
MOTOR_TYPE_N20             = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3  # (Default in many examples)

# 5) Chosen type & polarity
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0  # If speed is uncontrollable, try 1

# 6) Speeds to test
speed_forward  = [50, 50, 50, 50]   # All four motors forward at speed=50
speed_backward = [-50, -50, -50, -50]
speed_stop     = [0, 0, 0, 0]

def Motor_Init(bus):
    """Initialize motor type and encoder polarity."""
    bus.write_byte_data(MOTOR_ADDR, MOTOR_TYPE_ADDR, MotorType)
    time.sleep(0.2)
    bus.write_byte_data(MOTOR_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)
    time.sleep(0.2)

def read_battery(bus):
    """Read 2-byte battery voltage, return in millivolts."""
    data = bus.read_i2c_block_data(MOTOR_ADDR, ADC_BAT_ADDR, 2)
    voltage_mv = data[0] + (data[1] << 8)
    return voltage_mv

def read_encoder_counts(bus):
    """
    Read 16 bytes from MOTOR_ENCODER_TOTAL_ADDR,
    interpret as 4 int32 values (little-endian).
    """
    raw = bus.read_i2c_block_data(MOTOR_ADDR, MOTOR_ENCODER_TOTAL_ADDR, 16)
    # Convert list of bytes to a bytes object, then unpack as 4 little-endian int32
    counts = struct.unpack('<iiii', bytes(raw))
    return counts  # (Encode1, Encode2, Encode3, Encode4)

def write_fixed_speed(bus, speeds):
    """
    Write a list of 4 speed values to the 'MOTOR_FIXED_SPEED_ADDR'.
    Each speed is -100..100 (typical range).
    """
    # Convert each speed to an 8-bit signed value
    # The hardware expects 4 bytes for 4 motors
    data = []
    for s in speeds:
        data.append(s & 0xFF)
    bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, data)

def main():
    # Open I2C bus
    bus = smbus.SMBus(I2C_BUS)
    
    # Initialize motor settings
    Motor_Init(bus)

    # Read battery voltage & encoders once at the start
    v = read_battery(bus)
    print(f"Battery Voltage: {v} mV")
    
    enc = read_encoder_counts(bus)
    print(f"Encoders: {enc}")

    print("Moving forward at speed=50 for 3s...")
    write_fixed_speed(bus, speed_forward)
    time.sleep(3)

    print("Moving backward at speed=-50 for 3s...")
    write_fixed_speed(bus, speed_backward)
    time.sleep(3)

    print("Stopping motors.")
    write_fixed_speed(bus, speed_stop)
    
    # Read encoders one last time
    enc2 = read_encoder_counts(bus)
    print(f"Encoders after test: {enc2}")

    bus.close()

if __name__ == "__main__":
    main()
