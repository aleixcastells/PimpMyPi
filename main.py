import time
import os
from datetime import datetime
import pytz
import RPi.GPIO as GPIO
import numpy as np
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import json
from dotenv import load_dotenv
import csv
from twilio.rest import Client


# Load environment variables from .env file
load_dotenv()

# Time zone for Spain
timezone = pytz.timezone("Europe/Madrid")

### SETTINGS (CHANGE IN .env FILE) - - - - - - -

# Retrieve GPIO pin configurations
FAN_PIN = int(os.getenv("FAN_PIN", 17))
LED_1_PIN = int(os.getenv("LED_1_PIN", 23))
LED_2_PIN = int(os.getenv("LED_2_PIN", 24))
BTN_1_PIN = int(os.getenv("BTN_1_PIN", 27))
BTN_2_PIN = int(os.getenv("BTN_2_PIN", 22))

# Voltage thresholds
MIN_VOLTS = float(os.getenv("MIN_VOLTS", 12))
HIGH_VOLTS = float(os.getenv("HIGH_VOLTS", 12.6))  # Max Voltage
LOW_VOLTS = float(os.getenv("LOW_VOLTS", 10.8))  # Min Voltage

# Temperature thresholds (in degrees Celsius)
MIN_TEMP = float(os.getenv("MIN_TEMP", 30.0))
MAX_TEMP = float(os.getenv("MAX_TEMP", 50.0))

# Duty cycle thresholds (in percentage)
MIN_DUTY_CYCLE = float(os.getenv("MIN_DUTY_CYCLE", 40.0))
MAX_DUTY_CYCLE = float(os.getenv("MAX_DUTY_CYCLE", 100.0))

# LED settings
BLINK_INTERVAL = float(os.getenv("BLINK_INTERVAL", 0.5))

# Resistor settings
R1_VALUE = float(os.getenv("R1_VALUE", 10000.0))  # Ohms (10K)
R2_VALUE = float(os.getenv("R2_VALUE", 3300.0))  # Ohms (3.3K)

# Battery voltage range for charge percentage calculation
V_MIN = LOW_VOLTS  # 0%
V_MAX = HIGH_VOLTS  # 100%

# - - - - - - -

# Debounce settings for low voltage detection
LOW_VOLTAGE_COUNT = 0
LOW_VOLTAGE_THRESHOLD = 5  # Number of consecutive low readings required
CHECK_INTERVAL = 1  # Interval in seconds between checks

TWILIO_ENABLE = False
TWILIO_SID = ""
TWILIO_AUTH = ""
TWILIO_FROM = ""
TWILIO_TO = ""
TWILIO_CONTENT_SID = ""

# Retrieve TWILIO_ENABLE variables
if os.getenv("TWILIO_ENABLE", "FALSE").upper() == "TRUE":
    TWILIO_ENABLE = True
    TWILIO_SID = os.getenv("TWILIO_SID")
    TWILIO_AUTH = os.getenv("TWILIO_AUTH")
    TWILIO_FROM = os.getenv("TWILIO_FROM")
    TWILIO_TO = os.getenv("TWILIO_TO")

# Set up GPIO for fan control
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)

# Set up GPIO for LEDs
GPIO.setup(LED_1_PIN, GPIO.OUT)
GPIO.setup(LED_2_PIN, GPIO.OUT)

# Set up GPIO for buttons with pull-down resistors
GPIO.setup(BTN_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BTN_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize variables for LED blinking
led1_state = False
led2_state = False
led1_last_toggle_time = 0
led2_last_toggle_time = 0
led_blink_interval = BLINK_INTERVAL  # Blink interval in seconds

# Initialize variables for button states
BTN_1 = False
BTN_2 = False

# Set up PWM for fan control
pwm_frequency = 25  # 25Hz for fans
fan_pwm = GPIO.PWM(FAN_PIN, pwm_frequency)
fan_pwm.start(10)  # Start fan at 10% duty cycle

# Path for log files
log_folder = os.path.join(os.getcwd(), "logs")
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# Initialize I2C bus and ADC with error handling
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1  # Gain setting for the ADC (+/-4.096V)
    chan = AnalogIn(ads, ADS.P0)
    print("[INFO] ADC initialized successfully.")
except Exception as e:
    print(f"[ERROR] Failed to initialize ADC: {e}")

# Path to the script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Path to buttons.json
buttons_json_path = os.path.join(script_dir, "buttons.json")

# If buttons.json doesn't exist, create it with initial values
if not os.path.exists(buttons_json_path):
    with open(buttons_json_path, "w") as f:
        json.dump({"buttons": [False, False]}, f)


# Read the CPU temperature from the system.
def get_cpu_temperature():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        cpu_temp_raw = f.readline()
    return float(cpu_temp_raw) / 1000.0  # Convert to degrees Celsius


# Read the battery voltage
def read_battery_voltage():
    R1 = R1_VALUE
    R2 = R2_VALUE
    voltage_divider_ratio = (R1 + R2) / R2

    Vout = chan.voltage  # Voltage at the ADC input
    Vin = Vout * voltage_divider_ratio

    return Vin


# Calculate battery charge percentage
def calculate_battery_charge(voltage, v_min=V_MIN, v_max=V_MAX):
    if voltage <= v_min:
        return 0
    elif voltage >= v_max:
        return 100
    else:
        charge = (voltage - v_min) / (v_max - v_min) * 100
        return int(charge)


def calculate_fan_speed(cpu_temp):
    if cpu_temp > MAX_TEMP:
        return MAX_DUTY_CYCLE

    if cpu_temp < MIN_TEMP:
        return MIN_DUTY_CYCLE

    return np.interp(cpu_temp, [MIN_TEMP, MAX_TEMP], [MIN_DUTY_CYCLE, MAX_DUTY_CYCLE])


# Log CPU temperature, fan duty cycle, and battery voltage to a daily log file.
def log_status(cpu_temp, duty_cycle, battery_voltage, battery_charge, csv_writer):
    now = datetime.now(timezone)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M:%S")

    # Log file name based on current date
    log_file_path = os.path.join(log_folder, f"{date_str}.log")

    # Create or append to the log file
    with open(log_file_path, "a") as log_file:
        log_entry = f"[{time_str}] TEMP[{cpu_temp:.1f}] - FAN[{round(duty_cycle)}%] - BAT[{battery_voltage:.2f}V,{calculate_battery_charge(battery_voltage)}%] - BTNS[{BTN_1},{BTN_2}] - LEDS[{int(cpu_temp >= MAX_TEMP)},{int(battery_voltage < MIN_VOLTS)}]\n"
        log_file.write(log_entry)

    # Prepare data for CSV
    csv_data = {
        "Timestamp": time_str,
        "Temperature_C": cpu_temp,
        "Fan_Duty_Cycle_%": round(duty_cycle),
        "Battery_Voltage_V": battery_voltage,
        "Battery_Charge_%": battery_charge,
        "Button_1": BTN_1,
        "Button_2": BTN_2,
        "LED_1_State": int(cpu_temp >= MAX_TEMP),
        "LED_2_State": int(battery_voltage < MIN_VOLTS),
    }

    # Write to CSV
    csv_writer.writerow(csv_data)


def print_to_console(cpu_temp, duty_cycle, battery_voltage, battery_charge):
    console_entry = f"TEMP[{cpu_temp:.1f}°C] - FAN[{round(duty_cycle)}%] - BAT[{battery_voltage:.2f}V,{calculate_battery_charge(battery_voltage)}%] - BTNS[{BTN_1},{BTN_2}] - LEDS[{int(cpu_temp >= MAX_TEMP)},{int(battery_voltage < MIN_VOLTS)}]"
    print(console_entry)


# Function to control LEDs based on temperature and battery voltage
def control_leds(cpu_temp, battery_voltage):
    global led1_state, led2_state
    global led1_last_toggle_time, led2_last_toggle_time
    current_time = time.time()

    # LED 1 blinking (overheat)
    if cpu_temp >= MAX_TEMP:
        # Check if it's time to toggle the LED
        if current_time - led1_last_toggle_time >= led_blink_interval:
            # Toggle LED 1
            led1_state = not led1_state
            GPIO.output(LED_1_PIN, led1_state)
            led1_last_toggle_time = current_time

    else:
        # Ensure LED is off
        led1_state = False
        GPIO.output(LED_1_PIN, led1_state)

    # LED 2 blinking (low battery)
    if battery_voltage < MIN_VOLTS:
        # Check if it's time to toggle the LED
        if current_time - led2_last_toggle_time >= led_blink_interval:
            # Toggle LED 2
            led2_state = not led2_state
            GPIO.output(LED_2_PIN, led2_state)
            led2_last_toggle_time = current_time
    else:
        # Ensure LED is off
        led2_state = False
        GPIO.output(LED_2_PIN, led2_state)


def use_twilio(var1, var2, content_template_ssid):
    if TWILIO_ENABLE == True:
        content_variables_string = f'{"{"}"1":"{var1}","2":"{var2}"{"}"}'

        client = Client(TWILIO_SID, TWILIO_AUTH)
        message = client.messages.create(
            from_=TWILIO_FROM,
            content_sid=content_template_ssid,
            content_variables=f"{content_variables_string}",
            to=TWILIO_TO,
        )
        print(message.sid)


# Initialize a list to store temperature readings
last_temps = []
N = 3  # Number of readings to average

# Initialize CSV writer (will be updated inside the loop)
csv_writer = None
csv_file = None  # Initialize csv_file for scope


# Function to initialize CSV file
def initialize_csv(csv_file_path):
    file_exists = os.path.isfile(csv_file_path)
    csv_file = open(csv_file_path, mode="a", newline="")
    fieldnames = [
        "Timestamp",
        "Temperature_C",
        "Fan_Duty_Cycle_%",
        "Battery_Voltage_V",
        "Battery_Charge_%",
        "Button_1",
        "Button_2",
        "LED_1_State",
        "LED_2_State",
    ]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    if not file_exists:
        writer.writeheader()

    return csv_file, writer


# Function to handle low voltage scenarios
def handle_low_voltage(battery_voltage, battery_charge, csv_writer):
    print("Battery voltage below 10.8V! Initiating shutdown...")
    # Log the low voltage event
    now = datetime.now(timezone)
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M:%S")
    log_file_path = os.path.join(log_folder, f"{date_str}.log")
    with open(log_file_path, "a") as log_file:
        log_entry = f"[{time_str}] LOW VOLTAGE DETECTED: VOLTAGE[{battery_voltage:.2f}V] - CHARGE[{battery_charge}%]. SYSTEM SHUTTING DOWN.\n"
        log_file.write(log_entry)

    if TWILIO_ENABLE == True:
        use_twilio(
            battery_charge, battery_voltage, "HX4bfa83293466e6b98079698206a362dc"
        )
        content_variables_string = (
            f'{"{"}"1":"{battery_charge}","2":"{battery_voltage}"{"}"}'
        )

    # Prepare data for CSV
    csv_data = {
        "Timestamp": time_str,
        "Temperature_C": "N/A",
        "Fan_Duty_Cycle_%": "N/A",
        "Battery_Voltage_V": battery_voltage,
        "Battery_Charge_%": battery_charge,
        "Button_1": BTN_1,
        "Button_2": BTN_2,
        "LED_1_State": "N/A",
        "LED_2_State": "N/A",
    }

    # Write to CSV
    csv_writer.writerow(csv_data)

    # Execute system shutdown
    os.system("sudo shutdown -h now")


try:
    # 1. **Initial Startup Delay**
    print("[INFO] System is starting up. Waiting for voltage stabilization...")
    time.sleep(5)  # Wait for 5 seconds
    print("[INFO] Starting main loop.")

    last_log_time = time.time()  # Keep track of when to log to the file

    # Flag to ensure shutdown is triggered only once
    low_voltage_triggered = False

    temp_notified = False

    while True:
        # 2. **Read battery voltage with debug statements**
        battery_voltage = read_battery_voltage()

        # 3. **Calculate battery charge percentage with debug statements**
        battery_charge = calculate_battery_charge(battery_voltage)

        # 4. **Read the CPU temperature**
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

        # Control LEDs based on temperature and battery voltage
        control_leds(avg_cpu_temp, battery_voltage)

        # Check if temperature needs to be notified
        if cpu_temp >= MAX_TEMP + 4.0:
            if TWILIO_ENABLE == True and temp_notified == False:
                use_twilio(
                    round(cpu_temp),
                    round(current_duty_cycle),
                    "HX8846ba2e1c8a5a8de9b41de716c0efed",
                )
                temp_notified = True

        if cpu_temp < MAX_TEMP - 2.0:
            temp_notified = False

        # Read button states
        BTN_1 = GPIO.input(BTN_1_PIN)
        BTN_2 = GPIO.input(BTN_2_PIN)

        # Convert button states to boolean
        BTN_1_state = bool(BTN_1)
        BTN_2_state = bool(BTN_2)

        # Update buttons.json
        with open(buttons_json_path, "w") as f:
            json.dump({"buttons": [BTN_1_state, BTN_2_state]}, f)

        # Print temperature, fan speed, battery voltage, and charge to console every second
        print_to_console(
            avg_cpu_temp, current_duty_cycle, battery_voltage, battery_charge
        )

        # 5. **Low Voltage Handling with Debounce**
        if battery_voltage < LOW_VOLTS:
            LOW_VOLTAGE_COUNT += 1
            print(
                f"[DEBUG] Low voltage detected ({LOW_VOLTAGE_COUNT}/{LOW_VOLTAGE_THRESHOLD})"
            )
            if LOW_VOLTAGE_COUNT >= LOW_VOLTAGE_THRESHOLD and not low_voltage_triggered:
                # Determine log file and CSV file paths
                now = datetime.now(timezone)
                date_str = now.strftime("%Y-%m-%d")
                log_file_path = os.path.join(log_folder, f"{date_str}.log")
                csv_file_path = os.path.join(log_folder, f"{date_str}.csv")

                # Initialize CSV writer
                csv_file, csv_writer = initialize_csv(csv_file_path)

                # Handle low voltage (shutdown)
                handle_low_voltage(battery_voltage, battery_charge, csv_writer)

                # Close the CSV file after writing (although shutdown will stop the script)
                csv_file.close()

                # Set the flag to prevent multiple shutdowns
                low_voltage_triggered = True
        else:
            if LOW_VOLTAGE_COUNT > 0:
                print("[DEBUG] Voltage back to normal.")
            LOW_VOLTAGE_COUNT = 0

        # 6. **Log status every 60 seconds**
        if time.time() - last_log_time >= 60:
            # Determine log file and CSV file paths
            now = datetime.now(timezone)
            date_str = now.strftime("%Y-%m-%d")
            log_file_path = os.path.join(log_folder, f"{date_str}.log")
            csv_file_path = os.path.join(log_folder, f"{date_str}.csv")

            # Initialize CSV writer
            csv_file, csv_writer = initialize_csv(csv_file_path)

            # Log status to both log file and CSV
            log_status(
                avg_cpu_temp,
                current_duty_cycle,
                battery_voltage,
                battery_charge,
                csv_writer,
            )

            # Close the CSV file after writing
            csv_file.close()

            last_log_time = time.time()

        # 7. **Sleep for the defined check interval before the next reading**
        time.sleep(CHECK_INTERVAL)

except KeyboardInterrupt:
    print("\n[INFO] Script interrupted by user.")

finally:
    # Cleanup fan PWM and GPIO
    fan_pwm.stop()
    GPIO.cleanup()
    print("[INFO] GPIO cleanup completed.")
