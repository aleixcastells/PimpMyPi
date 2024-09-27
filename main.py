import time
import os
from datetime import datetime
import pytz
import RPi.GPIO as GPIO
import numpy as np

# Time zone for Spain
timezone = pytz.timezone("Europe/Madrid")

# Set pins
FAN_PIN = 23
LED_1_PIN = None
LED_2_PIN = None
BTN_1_PIN = None
BTN_2_PIN = None

# Set up GPIO for fan control
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)

# Set up PWM for fan control
pwm_frequency = 25  # 25Hz for fans
fan_pwm = GPIO.PWM(FAN_PIN, pwm_frequency)
fan_pwm.start(10)  # Start fan at 10% duty cycle

# Path for log files
log_folder = os.path.join(os.getcwd(), "logs")
if not os.path.exists(log_folder):
    os.makedirs(log_folder)


# Read the CPU temperature from the system.
def get_cpu_temperature():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        cpu_temp_raw = f.readline()
    return float(cpu_temp_raw) / 1000.0  # Convert to degrees Celsius


# Temperature thresholds (in degrees Celsius)
MIN_TEMP = 35.0  # Minimum temperature to start increasing fan speed
MAX_TEMP = 45.0  # Maximum temperature to reach maximum fan speed

# Duty cycle thresholds (in percentage)
MIN_DUTY_CYCLE = 10.0  # Minimum duty cycle (fan speed)
MAX_DUTY_CYCLE = 100.0  # Maximum duty cycle (fan speed)


def calculate_fan_speed(cpu_temp):
    global last_duty_cycle

    if cpu_temp > MAX_TEMP:
        return MAX_DUTY_CYCLE

    if cpu_temp < MIN_TEMP:
        return MIN_DUTY_CYCLE

    return np.interp(cpu_temp, [MIN_TEMP, MAX_TEMP], [MIN_DUTY_CYCLE, MAX_DUTY_CYCLE])


# Log CPU temperature and fan duty cycle to a daily log file.
# Get the current date and time in Spanish time
def log_temperature(cpu_temp, duty_cycle):
    now = datetime.now(timezone)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M:%S")

    # Log file name based on current date
    log_file_path = os.path.join(log_folder, f"{date_str}.log")

    # Create or append to the log file
    with open(log_file_path, "a") as log_file:
        log_entry = f"[{time_str}] Temp: {cpu_temp:.1f}°C, Fan: {round(duty_cycle)}%\n"
        log_file.write(log_entry)


def print_to_console(cpu_temp, duty_cycle):
    """Print CPU temperature and fan duty cycle to PM2 console."""
    now = datetime.now(timezone).strftime("%H:%M:%S")
    console_entry = f"[T:{cpu_temp:.1f}°C] [FAN:{round(duty_cycle)}%]"
    print(console_entry)


# Initialize a list to store temperature readings
last_temps = []
N = 3  # Number of readings to average

try:
    last_log_time = time.time()  # Keep track of when to log to the file

    while True:
        # Read the CPU temperature
        cpu_temp = get_cpu_temperature()

        # Append the current temperature to the list
        last_temps.append(cpu_temp)

        # Keep only the last N readings
        if len(last_temps) > N:
            last_temps.pop(0)

        # Calculate the average temperature
        avg_cpu_temp = sum(last_temps) / len(last_temps)

        # Calculate fan speed based on the average CPU temperature
        current_duty_cycle = calculate_fan_speed(avg_cpu_temp)
        fan_pwm.ChangeDutyCycle(current_duty_cycle)

        # Print temperature and fan speed to console every second
        print_to_console(avg_cpu_temp, current_duty_cycle)

        # Log the temperature and fan speed to the file every 60 seconds
        if time.time() - last_log_time >= 60:
            log_temperature(avg_cpu_temp, current_duty_cycle)
            last_log_time = time.time()

        # Sleep for 1 second before the next reading
        time.sleep(1)

except KeyboardInterrupt:
    pass  # Gracefully handle Ctrl+C

finally:
    # Cleanup fan PWM and GPIO
    fan_pwm.stop()
    GPIO.cleanup()
