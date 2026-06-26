#!/usr/bin/env python3
import os
import sys
import time

# Force gpiozero to use the modern lgpio driver for the Raspberry Pi 5
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder

# --- HARDWARE CONFIGURATION (Matching your exact layout) ---
# Left Motor Driver Control Pins
PWM_A_PIN = 12  # Physical Pin 32
AIN1_PIN  = 5   # Physical Pin 29
AIN2_PIN  = 6   # Physical Pin 31

# Right Motor Driver Control Pins
PWM_B_PIN = 13  # Physical Pin 33
BIN1_PIN  = 23  # Physical Pin 16
BIN2_PIN  = 24  # Physical Pin 18

# Encoder Feedback Pins (BCM Numbers)
LEFT_ENC_A = 17 # Physical Pin 11 (Yellow)
LEFT_ENC_B = 27 # Physical Pin 13 (Green)
RIGHT_ENC_A = 22 # Physical Pin 15 (Yellow)
RIGHT_ENC_B = 10 # Physical Pin 19 (Green)

print("⚙️ Initializing GPIO Zero devices...")

try:
    # Initialize Motor Outputs
    pwm_a = PWMOutputDevice(PWM_A_PIN, frequency=1000, initial_value=0)
    ain1  = DigitalOutputDevice(AIN1_PIN)
    ain2  = DigitalOutputDevice(AIN2_PIN)

    pwm_b = PWMOutputDevice(PWM_B_PIN, frequency=1000, initial_value=0)
    bin1  = DigitalOutputDevice(BIN1_PIN)
    bin2  = DigitalOutputDevice(BIN2_PIN)

    # Initialize Encoders
    left_encoder = RotaryEncoder(LEFT_ENC_A, LEFT_ENC_B, max_steps=0)
    right_encoder = RotaryEncoder(RIGHT_ENC_A, RIGHT_ENC_B, max_steps=0)

except Exception as e:
    print(f"❌ Initialization failed: {e}")
    print("Ensure no other ROS 2 nodes or scripts are running and claiming these pins!")
    sys.exit(1)

def stop_motors():
    ain1.off()
    ain2.off()
    bin1.off()
    bin2.off()
    pwm_a.value = 0.0
    pwm_b.value = 0.0

print("\n🚀 Hardware Diagnostic Test Initiated!")
print("Prop up the robot chassis so wheels spin freely in the air.")
print("Press CTRL+C at any time to safely abort.\n")
time.sleep(2)

try:
    # --- PHASE 1: FORWARD MOTION ---
    print("▶️ Phase 1: Driving FORWARD at 50% Speed...")
    ain1.on()
    ain2.off()
    bin1.on()
    bin2.off()
    pwm_a.value = 0.5
    pwm_b.value = 0.5

    # Monitor encoder values live for 3 seconds
    for _ in range(30):
        print(f"📊 Live Ticks -> Left Encoder: {left_encoder.steps} | Right Encoder: {right_encoder.steps}", end="\r")
        time.sleep(0.1)
    print("\n")

    # --- PHASE 2: BRAKE/STOP ---
    print("⏸️ Phase 2: STOPPING Motors for 2 Seconds...")
    stop_motors()
    time.sleep(2)
    print(f"📊 Stopped Ticks -> Left Encoder: {left_encoder.steps} | Right Encoder: {right_encoder.steps}\n")

    # --- PHASE 3: REVERSE MOTION ---
    print("◀️ Phase 3: Driving REVERSE at 50% Speed...")
    ain1.off()
    ain2.on()
    bin1.off()
    bin2.on()
    pwm_a.value = 0.5
    pwm_b.value = 0.5

    # Monitor encoder values live for 3 seconds
    for _ in range(30):
        print(f"📊 Live Ticks -> Left Encoder: {left_encoder.steps} | Right Encoder: {right_encoder.steps}", end="\r")
        time.sleep(0.1)
    print("\n")

except KeyboardInterrupt:
    print("\n🛑 Test interrupted by user.")

finally:
    print("🧹 Emergency Hardware Safety Shutdown Active. Cleaning rails...")
    stop_motors()
    left_encoder.close()
    right_encoder.close()
    print("🏁 Test complete. System safe.")
