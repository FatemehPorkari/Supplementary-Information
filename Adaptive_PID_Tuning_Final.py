# Adaptive_PID_Tuning_Final.py
import time
import board
import digitalio
import adafruit_max31865
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
from simple_pid import PID
from time import sleep

# --- Combined Full Cycle Profile ---
FULL_CYCLE_PROFILE = [
    # 1. Denaturation Steps
    (118.00,  9.13), 
    (118.00,  4.03), 
    (112.07,  2.96), 
    (107.48,  4.45), 

    # 2. Annealing Steps
    (48.00,  10.00),
    (49.00,  10.00),
    (50.02,  3.81),

    # 3. Final Extension Phase
    (85.37,   3.19),
    (80.22,   5.91),
    (75.69,   15.00),
]

# --- System Parameters ---
NUM_CYCLES = 24
READY_TEMP = 75.69
HEATER_PWM_PIN = 21
COOLER_PWM_PIN = 16
PWM_FREQUENCY = 1000
SAMPLE_TIME = 0.1  # seconds
TOLERANCE = 0.5    # °C

# --- Optimized Ramp Rates ---
HEATING_RAMP_RATE = 18.0  
COOLING_RAMP_RATE = 8.0   

# --- Advanced Control Parameters ---
BRAKING_OFFSET_PEAK_HEAT = 7.0    
BRAKING_OFFSET_DEEP_COOL = 30.0   
COAST_ACTIVATION_OFFSET = 2.5     
HYSTERESIS_THRESHOLD = 0.5        

# --- Optimized PID Coefficients ---
PID_RAMP_INITIAL_HEAT = (38.0, 1.2, 30.0)  
PID_RAMP_REHEAT       = (28.0, 1.5, 22.0)  
PID_RAMP_STEP_COOL    = (35.0, 1.2, 28.0)  
PID_RAMP_DEEP_COOL    = (30.0, 0.5, 25.0)  
PID_RAMP_STEP_HEAT    = (25.0, 2.8, 20.0)  
PID_SOAK_HIGH_T       = (32.0, 4.0, 28.0)  
PID_SOAK_MID_T        = (28.0, 2.0, 25.0)  
PID_SOAK_LOW_T        = (25.0, 1.0, 20.0)  
PID_GOTO_READY        = (35.0, 1.0, 30.0) 
PID_SOAK_READY        = (28.0, 4.5, 25.0) 

# --- State Machine Definitions ---
states = {
    'PREPARE': -3,
    'RETURN_TO_READY': -1,
    'DONE_CYCLE': -2,
    'START_PROFILE': 0,
    'COAST_TO_TARGET': 100
}
for i in range(len(FULL_CYCLE_PROFILE)):
    states[f'STEP_{i+1}_RAMP'] = (i * 2) + 1
    states[f'STEP_{i+1}_SOAK'] = (i * 2) + 2

# --- Hardware Initialization ---
def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HEATER_PWM_PIN, GPIO.OUT)
    GPIO.setup(COOLER_PWM_PIN, GPIO.OUT)
    heater = GPIO.PWM(HEATER_PWM_PIN, PWM_FREQUENCY)
    cooler = GPIO.PWM(COOLER_PWM_PIN, PWM_FREQUENCY)
    heater.start(0)
    cooler.start(0)
    return heater, cooler

def setup_sensors():
    spi = board.SPI()
    cs_peltier = digitalio.DigitalInOut(board.D12)
    cs_liquid = digitalio.DigitalInOut(board.D7)
    sensor_peltier = adafruit_max31865.MAX31865(spi, cs_peltier, wires=4, rtd_nominal=100.0, ref_resistor=430.0)
    sensor_liquid = adafruit_max31865.MAX31865(spi, cs_liquid, wires=4, rtd_nominal=100.0, ref_resistor=430.0)
    return sensor_peltier, sensor_liquid

def get_initial_temperature(sensor):
    try:
        return sensor.temperature
    except Exception as e:
        print(f"Warning: Could not read initial temperature: {e}. Defaulting to 25°C.")
        return 25.0

# --- Enhanced Control Logic ---
def run_thermal_cycler():
    heater_pwm, cooler_pwm = setup_gpio()
    peltier_sensor, liquid_sensor = setup_sensors()

    time_data, peltier_temp_data, liquid_temp_data, setpoint_data, pwm_output_data = [], [], [], [], []
    run_timestamp = time.strftime('%Y%m%d_%H%M%S')
    log_filename = f"full_cycle_log_T31_Final_{run_timestamp}.txt"
    plot_filename = f"full_cycle_plot_T31_Final_{run_timestamp}.png"

    with open(log_filename, "w") as log_file:
        log_file.write(f"Full Thermal Cycle Log (T31_Final)\nCycles: {NUM_CYCLES}\nReady Temp: {READY_TEMP}°C\n")
        log_file.write("="*80 + "\nTime(s),Cycle,Peltier(°C),Liquid(°C),Setpoint(°C),PWM(%),State\n")

        try:
            initial_temp = get_initial_temperature(peltier_sensor)
            pid = PID(setpoint=initial_temp, sample_time=SAMPLE_TIME, output_limits=(-100, 100))

            # --- Preparation Phase ---
            print(f"\n{'='*20} Preparing System: Moving to {READY_TEMP}°C {'='*20}")
            current_status = states['PREPARE']
            current_setpoint = initial_temp
            preparation_done = False

            while not preparation_done:
                peltier_temp = peltier_sensor.temperature
                liquid_temp = liquid_sensor.temperature
                if abs(peltier_temp - READY_TEMP) < TOLERANCE * 2 and current_status == states['PREPARE']:
                    print(f"\nReached Ready Temp vicinity. Soaking for 10s for stability...")
                    current_setpoint = READY_TEMP
                    pid.tunings = PID_SOAK_READY
                    soak_start_time = time.time()
                    while time.time() - soak_start_time < 10.0:
                         peltier_temp = peltier_sensor.temperature
                         liquid_temp = liquid_sensor.temperature
                         pid.setpoint = READY_TEMP
                         output = pid(peltier_temp)
                         if output > 0: 
                             heater_pwm.ChangeDutyCycle(min(output, 100))
                             cooler_pwm.ChangeDutyCycle(0)
                         else: 
                             heater_pwm.ChangeDutyCycle(0)
                             cooler_pwm.ChangeDutyCycle(min(abs(output), 100))
                         print(f"Soaking... Peltier: {peltier_temp:6.2f}°C | Liquid: {liquid_temp:6.2f}°C | Setpoint: {current_setpoint:6.2f}°C", end='\r')
                         sleep(SAMPLE_TIME)
                    print("\nSystem is stable and ready.                                          ")
                    preparation_done = True
                    continue

                pid.tunings = PID_GOTO_READY
                if READY_TEMP > peltier_temp:
                    current_setpoint = min(current_setpoint + HEATING_RAMP_RATE * SAMPLE_TIME, READY_TEMP)
                else:
                    current_setpoint = max(current_setpoint - COOLING_RAMP_RATE * SAMPLE_TIME, READY_TEMP)

                pid.setpoint = current_setpoint
                output = pid(peltier_temp)
                if output > 0: 
                    heater_pwm.ChangeDutyCycle(min(output, 100))
                    cooler_pwm.ChangeDutyCycle(0)
                else: 
                    heater_pwm.ChangeDutyCycle(0)
                    cooler_pwm.ChangeDutyCycle(min(abs(output), 100))
                print(f"Preparing... Peltier: {peltier_temp:6.2f}°C | Liquid: {liquid_temp:6.2f}°C | Setpoint: {current_setpoint:6.2f}°C", end='\r')
                sleep(SAMPLE_TIME)

            global_start_time = time.time()
            
            # --- Enhanced Cyclical Execution ---
            coast_target_step_index = 0
            for cycle_num in range(1, NUM_CYCLES + 1):
                print(f"\n{'='*20} Starting Cycle {cycle_num} of {NUM_CYCLES} {'='*20}")
                current_status = states['START_PROFILE']
                status_entry_time = time.time()
                current_setpoint = peltier_sensor.temperature
                output = 0

                while current_status != states['DONE_CYCLE']:
                    current_time_abs = time.time()
                    peltier_temp = peltier_sensor.temperature
                    liquid_temp = liquid_sensor.temperature
                    pid_gains = (0,0,0)

                    # State management
                    if current_status == states['START_PROFILE']:
                        pid_gains = PID_RAMP_INITIAL_HEAT if cycle_num == 1 else PID_RAMP_REHEAT
                        target_temp = FULL_CYCLE_PROFILE[0][0]
                        current_setpoint = min(current_setpoint + HEATING_RAMP_RATE * SAMPLE_TIME, target_temp)
                        if peltier_temp >= target_temp - BRAKING_OFFSET_PEAK_HEAT:
                            current_status = states['STEP_1_SOAK']
                            status_entry_time = current_time_abs
                            pid.reset()
                            current_setpoint = target_temp

                    elif current_status == states['RETURN_TO_READY']:
                        pid_gains = PID_GOTO_READY
                        current_setpoint = max(current_setpoint - COOLING_RAMP_RATE * SAMPLE_TIME, READY_TEMP)
                        if peltier_temp <= READY_TEMP + TOLERANCE:
                            current_status = states['DONE_CYCLE']

                    elif current_status == states['COAST_TO_TARGET']:
                        pid_gains = (0, 0, 0)
                        output = 0
                        target_temp, _ = FULL_CYCLE_PROFILE[coast_target_step_index]
                        current_setpoint = target_temp
                        
                        # Enhanced coasting logic with adaptive hysteresis
                        temp_diff = peltier_temp - target_temp
                        if (-HYSTERESIS_THRESHOLD <= temp_diff <= COAST_ACTIVATION_OFFSET and
                            abs(temp_diff) > HYSTERESIS_THRESHOLD/2):
                            print(f"\nCoasting complete. Engaging PID to catch and soak at {target_temp}°C.")
                            current_status = states[f'STEP_{coast_target_step_index + 1}_SOAK']
                            status_entry_time = current_time_abs
                            pid.reset()

                    else:
                        step_index = (current_status - 1) // 2
                        target_temp, soak_duration = FULL_CYCLE_PROFILE[step_index]

                        if current_status % 2 == 0: # SOAK state
                            if target_temp > 90.0:
                                pid_gains = PID_SOAK_HIGH_T
                            elif target_temp > 70.0:
                                pid_gains = PID_SOAK_MID_T
                            else:
                                pid_gains = PID_SOAK_LOW_T
                            current_setpoint = target_temp
                            if current_time_abs - status_entry_time >= soak_duration:
                                is_last_step = (step_index == len(FULL_CYCLE_PROFILE) - 1)
                                current_status = states['RETURN_TO_READY'] if is_last_step else states[f'STEP_{step_index + 2}_RAMP']
                                status_entry_time = current_time_abs
                                pid.reset()
                        else: # RAMP state
                            prev_temp = READY_TEMP if step_index == 0 else FULL_CYCLE_PROFILE[step_index - 1][0]
                            is_cooling = target_temp < prev_temp
                            if is_cooling:
                                is_deep_cool = (prev_temp - target_temp) > 40
                                pid_gains = PID_RAMP_DEEP_COOL if is_deep_cool else PID_RAMP_STEP_COOL
                                current_setpoint = max(current_setpoint - COOLING_RAMP_RATE * SAMPLE_TIME, target_temp)
                                if is_deep_cool and peltier_temp <= target_temp + BRAKING_OFFSET_DEEP_COOL:
                                    print(f"\nDeep cool brake triggered at {peltier_temp:.2f}°C. Entering COAST mode.")
                                    coast_target_step_index = step_index
                                    current_status = states['COAST_TO_TARGET']
                                elif peltier_temp <= target_temp + TOLERANCE:
                                    current_status = states[f'STEP_{step_index + 1}_SOAK']
                                    status_entry_time = current_time_abs
                                    pid.reset()
                                    current_setpoint = target_temp
                            else:
                                pid_gains = PID_RAMP_STEP_HEAT
                                current_setpoint = min(current_setpoint + HEATING_RAMP_RATE * SAMPLE_TIME, target_temp)
                                if peltier_temp >= target_temp - TOLERANCE:
                                    current_status = states[f'STEP_{step_index + 1}_SOAK']
                                    status_entry_time = current_time_abs
                                    pid.reset()
                                    current_setpoint = target_temp

                    # Control output
                    if current_status != states['COAST_TO_TARGET']:
                        if pid.tunings != pid_gains:
                            pid.tunings = pid_gains
                        pid.setpoint = current_setpoint
                        output = pid(peltier_temp)

                    # Apply control signals
                    if output > 0: 
                        heater_pwm.ChangeDutyCycle(min(output, 100))
                        cooler_pwm.ChangeDutyCycle(0)
                    else: 
                        heater_pwm.ChangeDutyCycle(0)
                        cooler_pwm.ChangeDutyCycle(min(abs(output), 100))

                    # Real-time monitoring
                    state_name = [name for name, val in states.items() if val == current_status][0]
                    print(f"Cycle {cycle_num:02d} | State: {state_name:<18} | Peltier: {peltier_temp:6.2f}°C | Liquid: {liquid_temp:6.2f}°C | Setpoint: {current_setpoint:6.2f}°C ", end='\r')

                    # Data logging
                    current_log_time = current_time_abs - global_start_time
                    time_data.append(current_log_time)
                    peltier_temp_data.append(peltier_temp)
                    liquid_temp_data.append(liquid_temp)
                    setpoint_data.append(current_setpoint)
                    pwm_output_data.append(output)
                    log_file.write(f"{current_log_time:.3f},{cycle_num},{peltier_temp:.3f},{liquid_temp:.3f},{current_setpoint:.3f},{output:.2f},{current_status}\n")
                    
                    sleep(SAMPLE_TIME)
                
                print(f"\nCycle {cycle_num} complete.                                                                                                ")
                if cycle_num < NUM_CYCLES:
                    print(f"Waiting for 5 seconds before next cycle...")
                    sleep(5.0)

            print("\nAll cycles completed successfully!")

        except KeyboardInterrupt:
            print("\nProcess interrupted by user.")
        finally:
            print("Cleaning up GPIO...")
            heater_pwm.stop()
            cooler_pwm.stop()
            GPIO.cleanup()
            print("Cleanup complete.")

    # --- Plotting Results ---
    if time_data:
        print("Plotting results...")
        plt.style.use('seaborn-v0_8-whitegrid')
        fig, ax1 = plt.subplots(figsize=(8, 6))
        ax1.plot(time_data, peltier_temp_data, 'b-', linewidth=2.5, label='Peltier Temp (°C)')
        ax1.plot(time_data, liquid_temp_data, 'g-', linewidth=2.0, label='Liquid Temp (°C)')
        ax1.plot(time_data, setpoint_data, 'r--', alpha=0.8, label='Setpoint (°C)')
        ax1.set_xlabel('Time (s)', fontsize=14)
        ax1.set_ylabel('Temperature (°C)', fontsize=14)
        ax1.legend(loc='upper left')
        ax1.grid(True, which='both', linestyle='--', linewidth=0.5)

        ax2 = ax1.twinx()
        ax2.fill_between(time_data, 0, pwm_output_data, 
                        where=[p > 0 for p in pwm_output_data], 
                        color='red', alpha=0.3, label='Heater Power (%)')
        ax2.fill_between(time_data, 0, pwm_output_data,
                        where=[p < 0 for p in pwm_output_data],
                        color='blue', alpha=0.3, label='Cooler Power (%)')
        ax2.set_ylabel('PID Output (%)', color='purple', fontsize=14)
        ax2.set_ylim(-105, 105)
        ax2.axhline(0, color='grey', linestyle='--', linewidth=0.8)
        ax2.legend(loc='upper right')

        fig.suptitle(f'Full Thermal Cycle (T31_Final) - {NUM_CYCLES} Cycles', 
                   fontsize=16, fontweight='bold')
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(plot_filename)
        print(f"Plot saved as {plot_filename}")
        plt.show()

if __name__ == '__main__':
    run_thermal_cycler()
