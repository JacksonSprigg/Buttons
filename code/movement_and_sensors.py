# Code by
# Jackson Sprigg
# Preet Patel

import RPi.GPIO as GPIO
import pigpio
import time
import math
import numpy as np
import sys

"""
Robot the thing
"""

############ Boot up robot control settings ##############
keep_robot_still = False         # False if you want the robot to move
angle_matters    = True          # If you care about the angle when arriving at a waypoint
##########################################################

######### PARAMETER TIME (Updated these from real world values) ##############

## ----------- Arena Parameters ------------##

# Positive rotaation is to the left (Curl with thumb out of page)
#    y 1200
#    ^
#    |
#    |
#    |
#    |
#    |   		      X  <-start(300,150) for M1 (0 deg is || x axis)
#    |  
#    +------------------------------> 1200 x

initial_x_pos = 950
initial_y_pos = 150
initial_angle = 90 * math.pi / (180)

# Define grid (For obstacle tracking)
grid_cell_size = 30    # Size of each grid cell in mm
grid_width     = 1200  # Width of the environment in mm    
grid_height    = 1200  # Height of the environment in mm   

# Create grid
grid_template            = [[0 for _ in range(grid_width // grid_cell_size)] for _ in range(grid_height // grid_cell_size)]
grid_timestamps_template = [[None for _ in range(grid_width // grid_cell_size)] for _ in range(grid_height // grid_cell_size)]
grid                     = grid_template[:]
grid_timestamps          = grid_timestamps_template[:]

# Init points
obstacles = [(0, 0, 0)]          # Obstacles -> list of tuples where each tuple is (x, y, time_stamp)
boundary  = (0, 0, 1200, 1200)   # (min_x, min_y, max_x, max_y)
waypoints = []

## ----------- PID Parameters ------------##
Kp_l = 0.08	    # Proportional left
Ki_l = 0     	# Integral left
Kp_r = Kp_l	    # Proportional right
Ki_r = Ki_l	    # Integral right

## ----------- Ultrasonic Parameters ------------##
detection_threshold = 20            # How many times do we need to see an obstacle in the same grid space before we decide it is real

obstacle_lifetime   = 100           # How long an obstacle stay in obstacles array (s)
grid_memory_time    = 10            # How long sensed data stays in our grid for (s)

ultra_pings_loop    = 2             # How many times we ping the ultrasonic sensors per loop 

minimum_distance    = 30                                    # Don't care about any signals closer than this as they are probably bad
timeout_distance    = 1000                                  # Won't wait for any reading further than this (mm) (Adjust)
timeout_on_us       = timeout_distance / (0.5 * 343000)     # Converted to how long we wait to hear ultras echo

exclusion_radius_around_waypoint  = 180                     # If an obstacle is this (mm) close to a waypoint of interest, ignore it

# Ultra positions relative to the center of robot
#                       1    2    3     4     5    6 
ultra_enable        = [False, True, False, False, True, False]
ultra_x_values      = [115,  130, 115, -115,  -130,  -115]
ultra_y_values      = [95,   0,   -95,  -95,   0,   95]

# Angle from center bot
ultra_angle_values  = [math.radians(90), math.radians(0), math.radians(-90), math.radians(180+90), math.radians(180), math.radians(180-90)]

## ----------- Wheel Parameters ------------##
robot = {
    'wheel_separation_mm': 225.5,
    'wheel_diam': 56,
    'gear_ratio': 74.83,
    'rotation_error_adjustment': 1.008,
    'wheel_circum': None,
    'dist_per_count': None
}

# Do some calculations
robot['wheel_circum']   = math.pi * robot['wheel_diam']
counts_per_rev          = 12 * robot['gear_ratio']
robot['dist_per_count'] = (robot['wheel_circum'] / counts_per_rev) * robot['rotation_error_adjustment']

## ----------- Control Parameters ------------##
control = {
    'hertz': 10000,        # PWM freq
    'loop_sleep': 0.05     # How long the main loop line sleeps for (0.05 seems to give motors a chance to pick the right setting)
}

tentacles = [(0, 0),         
             (0, -1.3), 
             (0, 1.3), 
             (0, -0.65), 
             (0, 0.65), 
             (140, 0), 
             (-140, 0), 
             (107, 0), 
             (-107, 0), 
             (70, 0), 
             (-70, 0), 
             (10, 0.74), 
             (10, -0.74), 
             (-10, 0.74), 
             (-10, -0.74), 
             (30, 0.94), 
             (30, -0.94),
             (-30, 0.94), 
             (-30, -0.94), 
             (100, 0.27), 
             (100, -0.27), 
             (-100, 0.27), 
             (-100, -0.27), 
             (125, 0.12), 
             (125, -0.12), 
             (-125, 0.12), 
             (-125, -0.12)
             ] ]

if keep_robot_still == True:
    tentacles = [(0, 0)]

## ----------- Waypoint accuracy Parameters ------------##
tolerance = {
    'dist': 20,                     # Distance before we conclude we're at waypoint (mm) tolerance['angle']
    'angle': math.radians(3),       # Angle tolerance
    'angle_matters': angle_matters  # Whether we care about angle or just want to get there (for M1 we just wanted to get there)
}          

## ----------- Cost Parameters ------------##
cost = {
    'steps'                : 12	 ,    # time_lookahead = steps * loop_time

    # Note that weights are not used anywhere. Cost function dynamically defines the weights
    'dist_weight'          : 1   ,    # Emphasis on distance
    'angle_facing_weight'  : 500 ,    # Emphasis on facing the direction dictated by the waypoint
    'angle_to_goal_weight' : 500 ,    # Emphasis on facing the (x,y) position of the goal itself

    # Unused vars
    'collision_proximity'  : 500 ,    # Not allowed closer than this (mm) to an object (note that it is from the center of our robot)
    'proximity_threshold'  : 75  ,    # How close we get to obstacles
}

###################################
## ----------- Setup ------------##
# Pin nums
# This are all chosen for being right next to each other and in groups of 6
ENCA_L = 8           
ENCB_L = 25         
ENCA_C = 24         
ENCB_C = 23         
ENCA_R = 18         
ENCB_R = 15          

PWM1_ml = 21         
PWM2_ml = 20         
PWM1_mc = 12         
PWM2_mc = 16         
PWM1_mr = 7          
PWM2_mr = 1          

Echo_1 = 13         
Trig_1 = 6         
Echo_2 = 5         
Trig_2 = 3          
Echo_3 = 0          
Trig_3 = 2         

Echo_4 = 27         
Trig_4 = 17         
Echo_5 = 10         
Trig_5 = 22         
Echo_6 = 11         
Trig_6 = 9          

button_1 = 26
button_3 = 19
button_4 = 4
button_6 = 14


# pig and GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

# Output - Triggers and PWM
for pin in [PWM1_mr, PWM2_mr, PWM1_ml, PWM2_ml, PWM1_mc, PWM2_mc, Trig_1, Trig_2, Trig_3, Trig_4, Trig_5, Trig_6]:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)

# Input - Echo and Buttons
for pin in [Echo_1, Echo_2, Echo_3, Echo_4, Echo_5, Echo_6, button_4, button_6, button_1, button_3]:
    GPIO.setup(pin, GPIO.IN)

# PIG for encoders only
for pin in [ENCA_L, ENCB_L, ENCA_R, ENCB_R, ENCA_C, ENCB_C]:
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_UP)

# Encoders interrupt reading
# Note that counters are opposite because of clock rotation is opposite on opposite sides
def right_wheel_interrupt(gpio, level, tick):
    global levA_R, levB_R, lastGpio_R, enc_count_right

    if gpio == ENCA_R:
        levA_R = level
    else:
        levB_R = level

    if gpio != lastGpio_R:  # Debounce
        lastGpio_R = gpio

        if gpio == ENCA_R and level == 1:
            if levB_R == 1:
                enc_count_right += 1
        elif gpio == ENCB_R and level == 1:
            if levA_R == 1:
                enc_count_right -= 1

def left_wheel_interrupt(gpio, level, tick):
    global levA_L, levB_L, lastGpio_L, enc_count_left

    if gpio == ENCA_L:
        levA_L = level
    else:
        levB_L = level

    if gpio != lastGpio_L:  # debounce
        lastGpio_L = gpio

        if gpio == ENCA_L and level == 1:
            if levB_L == 1:
                enc_count_left -= 1
        elif gpio == ENCB_L and level == 1:
            if levA_L == 1:
                enc_count_left += 1

def conveyer_interrupt(gpio, level, tick):
    global levA_C, levB_C, lastGpio_C, enc_count_conveyer
    if gpio == ENCA_C:
        levA_C = level
    else:
        levB_C = level

    if gpio != lastGpio_C:  # Debounce
        lastGpio_C = gpio

        if gpio == ENCA_C and level == 1:
            if levB_C == 1:
                enc_count_conveyer[0] += 1
        elif gpio == ENCB_C and level == 1:
            if levA_C == 1:
                enc_count_conveyer[0] -= 1
    
# Encoder interrupts (Pretty sure is on a daemon?)
cbA_left     = pi.callback(ENCA_L, pigpio.EITHER_EDGE, left_wheel_interrupt)
cbB_left     = pi.callback(ENCB_L, pigpio.EITHER_EDGE, left_wheel_interrupt)
cbA_right    = pi.callback(ENCA_R, pigpio.EITHER_EDGE, right_wheel_interrupt)
cbB_right    = pi.callback(ENCB_R, pigpio.EITHER_EDGE, right_wheel_interrupt)
cbA_conveyer = pi.callback(ENCA_C, pigpio.EITHER_EDGE, conveyer_interrupt)
cbB_conveyer = pi.callback(ENCB_C, pigpio.EITHER_EDGE, conveyer_interrupt)

# Start motors but turned off
motorright_PWM1     = GPIO.PWM(PWM1_mr, control['hertz'])
motorright_PWM2     = GPIO.PWM(PWM2_mr, control['hertz'])
motorleft_PWM1      = GPIO.PWM(PWM1_ml, control['hertz'])
motorleft_PWM2      = GPIO.PWM(PWM2_ml, control['hertz'])
motorconveyer_PWM1  = GPIO.PWM(PWM1_mc, control['hertz'])
motorconveyer_PWM2  = GPIO.PWM(PWM2_mc, control['hertz'])

motorright_PWM1.start(0)
motorright_PWM2.start(0)
motorleft_PWM1.start(0)
motorleft_PWM2.start(0)
motorconveyer_PWM1.start(0)
motorconveyer_PWM2.start(0)
#############################################################

## -----------------Tentacles------------------ ##

def is_point_near_obstacle(x, y, obstacles, proximity_threshold):
    for ox, oy, time_stamp in obstacles:
        distance = ((x - ox)**2 + (y - oy)**2)**0.5
        if distance < proximity_threshold:
            return True
    return False

def cost_function(tent_vel, tent_angular_vel, goal_x, goal_y, goal_angle, x, y, angle, current_vel, current_angular_vel, dt, steps=cost['steps']):
    '''
        cost_function function
        - takes a velocity and angular velocity, and predicts position and angle after 
          a specified number of time steps
        - compares to goal position and angle
        - returns discrepancy as a cost
    '''    

    # Calculate current angle to target and distance to target
    initial_dist_to_goal = ( (goal_x-x)**2 + (goal_y-y)**2 )**0.5
    angle_to_goal = math.atan2(goal_y-y, goal_x-x)
    err_angle_to_goal = abs(correct_angle(angle_to_goal - angle))
    err_angle_to_goal = min(err_angle_to_goal, math.pi - err_angle_to_goal)

    # If within 20mm of the goal, then don't worry about going towards the goal, and instead worry about facing the desired direction
    if initial_dist_to_goal < tolerance['dist']:
        if tent_vel != 0:
            return math.inf
        steps_in = math.ceil(steps *  0.1)  # only one step, to avoid overshooting. We are near the goal, so movement should be fine.
        dist_weight = 0
        angle_to_goal_weight = 0
        angle_facing_weight = 100
    # If far away from goal and not facing goal, then make sure robot is facing goal
    elif abs(err_angle_to_goal) > math.radians(8): 
        # Do not allow forwards movement
        steps_in = steps * 1
        dist_weight = 0.5
        angle_to_goal_weight = 200
        angle_facing_weight = 0
    # If far away from goal and facing goal, then allow robot to move forward and rotate towards goal
    else:
        steps_in = steps * 1
        dist_weight = 2
        angle_to_goal_weight = 500      # gives comparable weight to both distance and angle_to_goal
        angle_facing_weight = 0

    # Estimate position after 'steps' time steps, where each is a duration of 'dt'
    for _ in range(steps_in):      # CHECK PP can be done with a single calculation
        x = x + dt * tent_vel * math.cos(angle)
        y = y + dt * tent_vel * math.sin(angle)
        angle = (angle + tent_angular_vel * dt)

    final_dist_to_goal = ((goal_x - x)**2 + (goal_y - y)**2)**0.5

    # Estimate the error in estimated angle and goal angle
    err_angle_facing = correct_angle(goal_angle - angle)
    err_angle_to_goal = abs(correct_angle(angle_to_goal - angle))
    err_angle_to_goal = min(err_angle_to_goal, math.pi - err_angle_to_goal)
    
    return (dist_weight * final_dist_to_goal + 
            angle_to_goal_weight * abs(err_angle_to_goal) +
            angle_facing_weight * abs(err_angle_facing)
            )

def plan_tentacle(goal_x, goal_y, goal_th, x, y, angle, current_vel, current_angular_vel, dt, tentacles=tentacles):
    costs = []
    for vel, angular_vel in tentacles:
        costs.append(cost_function(vel, angular_vel, goal_x, goal_y, goal_th, x, y, angle, current_vel, current_angular_vel, dt))
    best_idx = np.argmin(costs)
    return tentacles[best_idx]

def correct_angle(input_angle):
    # This below line also works
    # math.atan2(math.sin(input_angle), math.cos(input_angle)) 
    return (input_angle + math.pi) % (2 * math.pi) - math.pi
############################################################

## ----------------- Ultrasonic ------------------------- ##

# Sensor pin dict
sensors = {
    'sensor_1': {'trigger': Trig_1, 'echo': Echo_1},
    'sensor_2': {'trigger': Trig_2, 'echo': Echo_2},
    'sensor_3': {'trigger': Trig_3, 'echo': Echo_3},
    'sensor_4': {'trigger': Trig_4, 'echo': Echo_4},
    'sensor_5': {'trigger': Trig_5, 'echo': Echo_5},
    'sensor_6': {'trigger': Trig_6, 'echo': Echo_6}
}


# Read a distance from an ultra
def get_distance(trigger_pin, echo_pin):

    # Send 10us ultrasonic pulse
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)					    
    GPIO.output(trigger_pin, GPIO.LOW)

    # Defined here incase they get referenced before assignment
    start_time = time.time()
    end_time = time.time()
    timeout_time = start_time + timeout_on_us

    while GPIO.input(echo_pin) == 0 and time.time() < timeout_time:
        start_time = time.time()

    if time.time() >= timeout_time:
        return None

    timeout_time = start_time + timeout_on_us

    while GPIO.input(echo_pin) == 1 and time.time() < timeout_time:
        end_time = time.time()
        
    if time.time() >= timeout_time:
        return None
    
    pulse_duration = end_time - start_time

    # Calculate distance in mm
    distance = 0.5 * 343000 * pulse_duration
    distance = round(distance,2)

    if distance < minimum_distance:
        return None
    
    return distance

# Filter for samples hits to estimate a distance
def get_filtered_distance(trigger_pin, echo_pin, samples=25):
    # Collect data
    distances = []
    for _ in range(samples):
        new_distance = get_distance(trigger_pin, echo_pin)
        time.sleep(0.01)
        if new_distance:
            distances.append(new_distance)
    # Sort data
    distances.sort()

    samples = len(distances)
    # Trim data: Remove the top and bottom 25% of readings to filter out outliers
    trim_percent = 25
    trim_count = int(samples * trim_percent / 100)
    trimmed_distances = distances[trim_count:-trim_count]
    if len(trimmed_distances) > 0:
        # Calculate average of the remaining data
        average_distance = sum(trimmed_distances) / len(trimmed_distances)
        
        return average_distance
    else:
        return 0

# Loops through all sensors puts their respective values into an array
def get_all_distance_looped():
    distances = []
    sensor_vals = list(sensors.values())
    for i in range(len(sensor_vals)):
        sensor = sensor_vals[i]
        distances.append(get_distance(sensor['trigger'], sensor['echo']))
    return distances

## Not sure if this is right?? ##  
def obstacle_absolute_position(distance, current_x, current_y, current_angle, ultra_x, ultra_y, ultra_angle):

    obs_x_pos = current_x + math.cos(current_angle)*ultra_x - math.sin(current_angle)*ultra_y + math.cos(current_angle+ultra_angle)*distance
    obs_y_pos = current_y + math.sin(current_angle)*ultra_x + math.cos(current_angle)*ultra_y + math.sin(current_angle+ultra_angle)*distance
    
    return obs_x_pos, obs_y_pos

# Get a grid square based on float pos values
def position_to_grid(x, y):

    # If it's withinn half cell size to the wall it's probably just the wall so don't worry
    if x < grid_cell_size * 4 or x > grid_width - grid_cell_size * 4 or y < grid_cell_size * 4 or y > grid_height - grid_cell_size * 4:
        return None, None
    
    grid_x = int( x // grid_cell_size)
    grid_y = int( y // grid_cell_size)
    return grid_x, grid_y

# Add an obstacle point to the grid space it was seen in
def update_grid_with_obstacle(x, y):
    
    grid_x, grid_y = position_to_grid(x, y)

    # Check if the position is out of bounds
    if grid_x is None or grid_y is None:
        return
    
    grid[grid_y][grid_x] += 1
    grid_timestamps[grid_y][grid_x] = time.time()

# If we see detection_threshold obstacles in a grid we consider it to be confirmed obstacle
def detect_confirmed_obstacles():
    confirmed_obstacles = []
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] >= detection_threshold:
                confirmed_obstacles.append((x * grid_cell_size, y * grid_cell_size))
    return confirmed_obstacles

# Reset all grid obstacle counts to zero
def prune_old_grid_entries(grid_memory_time):
    current_time = time.time()
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid_timestamps[y][x] and current_time - grid_timestamps[y][x] > grid_memory_time:
                grid[y][x] = 0
                grid_timestamps[y][x] = None

# If an obstacle has been in out obstacle list for > obstacle_lifetime, get rid of it			
def prune_old_obstacles(obstacles, obstacle_lifetime):
    current_time = time.time()
    fresh_obstacles = []

    for x, y, timestamp in obstacles:
        age_of_obstacle = current_time - timestamp
        if age_of_obstacle <= obstacle_lifetime:
            fresh_obstacles.append((x, y, timestamp))

    return fresh_obstacles

# Ping ultras and update grid obstacle counters
def add_ultras_to_grid(raw_distances, current_x_pos, current_y_pos, current_angle, obstacles):
    for idx, distance in enumerate(raw_distances):
        if ultra_enable[idx]:
            if distance is None:
                continue
                
            ultra_x = ultra_x_values[idx]
            ultra_y = ultra_y_values[idx]
            ultra_angle = ultra_angle_values[idx]
            
            obs_x, obs_y = obstacle_absolute_position(distance, current_x_pos, current_y_pos, current_angle, ultra_x, ultra_y, ultra_angle)
            update_grid_with_obstacle(obs_x, obs_y)

# Goes through grid and add new obstacles to out obstacle list
def new_obstacle_check_and_add(waypoints):
    confirmed_obstacles = detect_confirmed_obstacles()
    for obs_x, obs_y in confirmed_obstacles:
        if not any(o[0] == obs_x and o[1] == obs_y for o in obstacles):
            flag = False
            for each_waypoint in waypoints:
                if each_waypoint[3] and (abs(obs_x - each_waypoint[0]) < exclusion_radius_around_waypoint) and (abs(obs_y-each_waypoint[1]) < exclusion_radius_around_waypoint):
                   flag = True 
            if flag:
                break
            current_time = time.time()
            obstacles.append((obs_x, obs_y, current_time))

            # Debugging print statement
            print(f"New obstacle detetcted at x={obs_x}, y={obs_y}")

###################################################################

## ------ PI and Motor controls -----------##

# Pi control the duty cycle to keep desired vels
def duty_cycle_pi_control(duty_cycle, speed_desired, speed_measured, Kp, Ki, err_sum):
    duty_cycle = min(  (max(-100, duty_cycle + Kp * (speed_desired - speed_measured) + Ki * err_sum))  ,100)
    err_sum = err_sum + speed_desired - speed_measured
    return duty_cycle, err_sum

def motors_off():
    duty_cycle['left']  = 0
    duty_cycle['right'] = 0
    motorleft_PWM1.ChangeDutyCycle(0)
    motorleft_PWM2.ChangeDutyCycle(0)
    motorright_PWM1.ChangeDutyCycle(0)
    motorright_PWM2.ChangeDutyCycle(0)

def set_motor_left(duty_cycle):
    # Sets motors at a duty cycle between (-100, 100)
    if duty_cycle >= 0:
        motorleft_PWM1.ChangeDutyCycle(duty_cycle)
        motorleft_PWM2.ChangeDutyCycle(0)
    else:
        motorleft_PWM1.ChangeDutyCycle(0)
        motorleft_PWM2.ChangeDutyCycle(abs(duty_cycle))

def set_motor_right(duty_cycle):
    # Sets motors at a duty cycle between (-100, 100)
    if duty_cycle >= 0:
        motorright_PWM1.ChangeDutyCycle(duty_cycle)
        motorright_PWM2.ChangeDutyCycle(0)
    else:
        motorright_PWM1.ChangeDutyCycle(0)
        motorright_PWM2.ChangeDutyCycle(abs(duty_cycle))

## -----------------Calibration------------------ ##
## Vars for calibration
time_for_confirmation = 0.1    # How long we need to be flat against the wall to be content in our new position (s)
cal_speed = 10

def calibrate(area):
    scale = 0.98
    difference_threshold = 30
    success_threshold = 0.5 	# How long we need to be on the wall for (s)
    loop_sleeper 	  = 0.1		# Sleep between each action

    ##### VARIABLEs HERE CHECK THEM #####
    ultra_to_center_x = 108
    total_x_distance  = 1200 - 2*ultra_to_center_x		# What the added distance on either side of sensors should be
    flash_every_x_loopsleep = 5 # How many loop sleepers to wait between flashes
    success_counter   = 0
    flash_counter	  = 0
    cal_start_time = time.time()
    while (success_counter < (success_threshold * (1 / loop_sleeper))) and (time.time()-cal_start_time<10):
        time.sleep(loop_sleeper)
        
        if area == "loading":
            ## - Flashing lights :) - ##
            # Turn buttons into outputs for flashing
            for pin in [button_1, button_3]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, 0)
                
            if flash_counter % flash_every_x_loopsleep == 0 and flash_counter % (flash_every_x_loopsleep * 2) != 0:
                GPIO.output(button_1, 1)
                GPIO.output(button_3, 0)
                
            if flash_counter % (flash_every_x_loopsleep * 2) == 0:
                GPIO.output(button_1, 0)
                GPIO.output(button_3, 1)
            ##############################
        
            # Check what buttons are doing
            back_left   = GPIO.input(button_4)
            back_right  = GPIO.input(button_6)
            if (not back_left) and (not back_right):
                duty_cycle['left']  = (-cal_speed)
                duty_cycle['right'] = (-cal_speed)
                mode = 'reversing'
                success_counter = 0 
            elif back_left and (not back_right):
                duty_cycle['left']  = (-cal_speed)
                duty_cycle['right'] = (0)
                mode = 'anticlockwising'
                success_counter = 0
            elif (not back_left) and back_right:
                duty_cycle['left']  = (0)
                duty_cycle['right'] = (-cal_speed)
                mode = 'clockwising'
                success_counter = 0
            elif back_left and back_right:
                duty_cycle['left']  = (0)
                duty_cycle['right'] = (0)
                success_counter += 1
                mode = 'stopped'
                
        if area == "delivering":
            ## - Flashing lights :) - ##
            # Turn buttons into outputs for flashing
            for pin in [button_4, button_6]:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, 0)
                
            if flash_counter % flash_every_x_loopsleep == 0 and flash_counter % (flash_every_x_loopsleep * 2) != 0:
                GPIO.output(button_4, 1)
                GPIO.output(button_6, 0)
                
            if flash_counter % 10 == 0:
                GPIO.output(button_4, 0)
                GPIO.output(button_6, 1)
            ##############################
            
            # Check what buttons are doing
            front_left  = GPIO.input(button_1)
            front_right = GPIO.input(button_3)
            if (not front_left) and (not front_right):
                duty_cycle['left']  = (cal_speed)
                duty_cycle['right'] = (cal_speed)
                mode = 'forward'
                success_counter = 0
            elif front_left and (not front_right):
                duty_cycle['left']  = (0)
                duty_cycle['right'] = (cal_speed)
                mode = 'anticlockwising'
                success_counter = 0
            elif (not front_left) and front_right:
                duty_cycle['left']  = (cal_speed)
                duty_cycle['right'] = (0)
                mode = 'clockwising'
                success_counter = 0
            elif front_left and front_right:
                duty_cycle['left']  = (0)
                duty_cycle['right'] = (0)
                success_counter += 1
                mode = 'stopped'
        
        # Run motors based on buttons	
        set_motor_left(duty_cycle['left'])
        set_motor_right(duty_cycle['right'])
        # counter used for the flashes flashing lights
        flash_counter += 1
    duty_cycle['left']  = (0)
    duty_cycle['right'] = (0)
    set_motor_left(duty_cycle['left'])
    set_motor_right(duty_cycle['right'])
    ##--------Reset location------------##
    # If x isn't in an allowed tolerance accuracy then we just don't worry about it on that delivery

    if area == "loading":
        current['y'] = 145
        previous['y'] = current['y']

        current['angle'] = 90 * (math.pi / 180)
        previous['angle'] = current['angle']
        
    if area == "delivering":
        current['y']  = 1055
        previous['y'] = current['y']

        current['angle']  = 90 * (math.pi / 180)
        previous['angle'] = current['angle']
    
    if current['x'] < 400:
        # Get distances on either side
        distance_1 = get_filtered_distance(sensors['sensor_1']['trigger'], sensors['sensor_1']['echo'])     # 10mm
        distance_6 = get_filtered_distance(sensors['sensor_6']['trigger'], sensors['sensor_6']['echo'])     # 11mm
        diff = distance_1 - distance_6
        print(f"Attempt to Localise X: |Front = {distance_1:.1f}|Back = {distance_6:.1f}|Diff = {diff=:.1f}")
        
        # Check that the distance is within a tolerance
        if diff < difference_threshold:        # 
            current['x']  = scale*min(distance_1, distance_6) + ultra_to_center_x
            previous['x'] = current['x']
            print("-> Successfully localised X")
    elif current['x'] > 800:
        # Get distances on either side
        distance_3 = get_filtered_distance(sensors['sensor_3']['trigger'], sensors['sensor_3']['echo'])     # 10mm
        distance_4 = get_filtered_distance(sensors['sensor_4']['trigger'], sensors['sensor_4']['echo'])     # 11mm
        diff = distance_3 - distance_4
        print(f"Attempt to Localise X: |Front = {distance_3:.1f}|Back = {distance_4:.1f}|Diff = {diff=:.1f}")
        
        # Check that the distance is within a tolerance
        if diff < difference_threshold:        # 
            current['x']  = 1200 - scale*min(distance_3, distance_4) - ultra_to_center_x
            previous['x'] = current['x']
            print("-> Successfully localised X")
    print(f"-> Position reset: ({current['x']:.1f}, {current['y']:.1f}, {current['angle'] * (180/math.pi):.1f})")

    ############################################################

    # Reset make all pins inputs again
    for pin in [button_4, button_6, button_1, button_3]:
        GPIO.setup(pin, GPIO.IN)
#####################################################

## ------------ UGLY FUNCTION ------------- ##
# Jackson thinks this needs to be cleaned, but not priority
def check_front():
    # Parameters, but only used in this function :(
    self_width  = 300
    self_length = 260

    rel_front_right_x  = self_length/2
    rel_front_right_y  = -self_width/2
    front_box_width    = -2 * rel_front_right_y        
    front_box_length   = 150 
    away_from_boundary = 190

    box_x = current['x'] + math.cos(current['angle'])*rel_front_right_x - math.sin(current['angle'])*rel_front_right_y
    box_y = current['y'] + math.sin(current['angle'])*rel_front_right_x + math.cos(current['angle'])*rel_front_right_y

    obst_in_box_count = 0
    for obs_x, obs_y,_ in obstacles:
        rel_x = obs_x - box_x
        rel_y = obs_y - box_y
        rotated_x = round(math.cos(-current['angle'])*rel_x - math.sin(-current['angle'])*rel_y,2)
        rotated_y = round(math.sin(-current['angle'])*rel_x + math.cos(-current['angle'])*rel_y,2)
        if (0 <= rotated_x <= front_box_length) and (0 <= rotated_y <= front_box_width):
            obst_in_box_count += 1
    if obst_in_box_count >= 1:
        for jump_direction in [1, -1]:      # 1: left, -1: right
        # Add waypoints
            lateral_movement = self_width*0.8
            new_x_1 = round(current['x'] - math.sin(current['angle'])*lateral_movement * jump_direction,2)
            new_y_1 = round(current['y'] + math.cos(current['angle'])*lateral_movement * jump_direction,2)
            new_angle_1 = current['angle'] + math.radians(90*jump_direction)
            move_forward_length = min((2*self_length), distance_to_waypoint)
            new_x_2 = round(current['x'] + math.cos(current['angle'])*move_forward_length - math.sin(current['angle'])*lateral_movement * jump_direction,2)
            new_y_2 = round(current['y'] + math.sin(current['angle'])*move_forward_length + math.cos(current['angle'])*lateral_movement * jump_direction,2)
            new_angle_2 = current['angle']

            if (away_from_boundary <= new_x_1 <= boundary[2]-away_from_boundary) and (away_from_boundary <= new_y_1 <= boundary[3]-away_from_boundary) and (away_from_boundary <= new_x_2 <= boundary[2]-away_from_boundary) and (away_from_boundary <= new_y_2 <= boundary[3]-away_from_boundary):
                print(
    f'''\nReprogamming route\n    Obstacles: {obstacles}    
    Current: ({current['x']}, {current['y']}, {current['angle']})
    Turning: {'left' if (jump_direction == 1) else 'right'}
    New W1: ({new_x_1}, {new_y_1}, {round(math.degrees(new_angle_1),1)}°)
    New W2: ({new_x_2}, {new_y_2}, {round(math.degrees(new_angle_2),1)}°)
    ''')
                motors_off()
                # Wait if this is a main waypoint, continue immediately if it is a added waypoint
                # time.sleep(0)
                return [(new_x_1, new_y_1, new_angle_1, False), (new_x_2, new_y_2, new_angle_2, False)]
        
        motors_off()
        time.sleep(2)
    return []
#########################################################

## ---------- Colour format for debugging ---------- ##
green_background = "\033[42m"
red_background   = "\033[41m"
reset_format     = "\033[0m"
def color(num, _color = None):
    if _color == 'green_digit':
        return green_background + f"{num}" + reset_format
    elif _color == 'red_digit':
        return red_background + f"{num}" + reset_format    
    else:
        if num > 0:
            return green_background + "+"+f"{num:<5.1f}" + reset_format
        elif num < 0:
            return red_background + "-"+f"{-num:<5.1f}" + reset_format
        else:
            return "0.0"		
###########################################################

## -------------Cleanup function------------------------- ##
def cleanup(debug):
    motors_off()
    cbA_left.cancel()
    cbB_left.cancel()
    cbA_right.cancel()
    cbB_right.cancel()
    pi.stop()
    GPIO.cleanup()

    # Printing a grid hit map for debugging #
    for i in reversed(grid):
        for j in i:
            if j == 0:
                print(f" .",end=" ") if debug else None
            else:
                print(f"{j:2g}",end=" ") if debug else None
        print()

    print("Finished")
    sys.exit()
################################################################

## ----------- Initialisations ------------##
## Positional ##
current = {
    'x': initial_x_pos,
    'y': initial_y_pos,
    'angle': initial_angle
}

## Previous positional ##
previous = {
    'x': initial_x_pos,
    'y': initial_y_pos,
    'angle': initial_angle
}

## Time ##
times = {
    'current': time.time(),
    'previous_updates': time.time(),
    'previous_tentacles': time.time()
}

## Encoders ##
# Not added to dictionary due to high frequency of usage. 
enc_count_right    = 0
enc_count_left     = 0
enc_count_conveyer = [0]    # list because immutable objects cannot be imported to another file (cool trick Preet)
levA_L           = 0
levB_L           = 0
levA_R           = 0
levB_R           = 0
levA_C           = 0
levB_C           = 0
lastGpio_L       = None
lastGpio_R       = None
lastGpio_C       = None

## PI/Motor Control ##
duty_cycle = {
    'left': 0,
    'right': 0,
    'left_err_sum': 0,
    'right_err_sum': 0
}
vel = {
    'left':0,
    'right': 0,
    'current': 0,
    'angular': 0
}

distance_to_waypoint = 100
##################################################

## Main Loop ##
def move(waypoints, debug, collision_check):
    global obstacles, grid, grid_timestamps, distance_to_waypoint
    collision_count = 0
    collision_limit = 3
    collision_forsight = 150
	# Try statement to just catch Keyboard Interrupts
    try:
        # Some initialisations moved here because they are only used in here
        goal_x, goal_y, goal_angle, is_main = waypoints[0]
        previous_right_distance_mm = enc_count_right * robot['dist_per_count']
        previous_left_distance_mm  = enc_count_left  * robot['dist_per_count']       

        while waypoints:
            # Calculate distance and angle error
            distance_to_waypoint = math.sqrt((goal_x - current['x'])**2 + (goal_y - current['y'])**2)
            angle_difference     = correct_angle(goal_angle - current['angle'])

            # If close enough to waypoint then update to next
            if distance_to_waypoint < tolerance['dist']:
                # If angle doesn't matter or angle is good enough
                if (not tolerance['angle_matters']) or (abs(angle_difference) < tolerance['angle']):
                    ## Pause on waypoint ##
                    motors_off()
                    print(f"-> Waypoint ({goal_x:.1f}, {goal_y:.1f}, {goal_angle*(180/math.pi):.1f}) found")
                    # Wait if this is a main waypoint, continue immediately if it is an added waypoint
                    if is_main:
                        time.sleep(0.05)

                    # Pop to get next waypoint
                    waypoints.pop(0)

                    # If there are no more waypoints, break out of the loop
                    if not waypoints:
                        break
                    
                    # Set next waypoint    
                    goal_x, goal_y, goal_angle, is_main = waypoints[0]
					
					# Update
                    distance_to_waypoint = math.sqrt((goal_x - current['x'])**2 + (goal_y - current['y'])**2)
                    angle_difference = correct_angle(goal_angle - current['angle'])

            # Keep timing consistent by sleeping (Makes every tentacle time step = loop_sleep)
            times['current'] = time.time()
            time_since_loop_tentacles = times['current'] - times['previous_tentacles']
            sleepy = max(0, control['loop_sleep'] - time_since_loop_tentacles - 0.0001)
            time.sleep(sleepy)
            
            # Do cost analysis to decide what speed we should be going
            desired_vel, desired_angular_vel = plan_tentacle(goal_x, goal_y, goal_angle, current['x'], current['y'], current['angle'], vel['current'], vel['angular'], control['loop_sleep'])
            
            # Start timer again
            times['previous_tentacles'] = time.time()

            # Convert to vel and ang_vel to left and right wheel vels respectivley
            desired_vel_left  = desired_vel  - 0.5 * desired_angular_vel * robot['wheel_separation_mm']
            desired_vel_right = desired_vel  + 0.5 * desired_angular_vel * robot['wheel_separation_mm']
            
            # PI control to match duty cycles to desired wheel velocities (LOOK INTO DIRECT MAPPING)
            duty_cycle['left'], duty_cycle['left_err_sum']   = duty_cycle_pi_control(duty_cycle['left'], desired_vel_left, vel['left'], Kp_l, Ki_r, duty_cycle['left_err_sum'])
            duty_cycle['right'], duty_cycle['right_err_sum'] = duty_cycle_pi_control(duty_cycle['right'], desired_vel_right, vel['right'], Kp_r, Ki_r, duty_cycle['right_err_sum'])
            
            # Set motors to calculated duty cycles
            set_motor_left(duty_cycle['left'])
            set_motor_right(duty_cycle['right'])
            # set_motor_left(0)
            # set_motor_right(0)

            ##-------- Current position and angle tracking ------------ ##
            # Update timer for this section (Different timer to the other section)
            times['current'] = time.time()
            time_since_loop_updates = times['current'] - times['previous_updates']
            times['previous_updates']  = time.time()        # Why is it redefined so far below PP?

            # Total distance
            right_distance_mm = enc_count_right * robot['dist_per_count']
            left_distance_mm  = enc_count_left * robot['dist_per_count']
            
            # Current vel of wheels (small angle approximation)
            vel['right'] = (right_distance_mm - previous_right_distance_mm) / time_since_loop_updates       # PP shouldnt this be calculated much earlier?
            vel['left']  = (left_distance_mm  - previous_left_distance_mm) / time_since_loop_updates
            
            # Current description of system
            vel['current'] = (vel['right'] + vel['left']) / 2
            vel['angular'] = (vel['right'] - vel['left']) / robot['wheel_separation_mm']

            # Current position and angle
            current['x'] = previous['x'] + vel['current'] * math.cos(current['angle']) * time_since_loop_updates
            current['y'] = previous['y'] + vel['current'] * math.sin(current['angle']) * time_since_loop_updates
            current['angle'] = previous['angle'] + vel['angular'] * time_since_loop_updates
			#################################################################

            ## -------- Updates for next loop ------- ##
            previous_right_distance_mm = right_distance_mm
            previous_left_distance_mm = left_distance_mm
            previous['x'] = current['x']
            previous['y'] = current['y']
            previous['angle'] = current['angle']
            times['previous_updates']  = time.time()
            ############################################

            ## ----- Ultrasonic Obstacle detection ----- ##
            # Ping if we are moving and we are facing our desired waypoint (i.e. probably not turning sharply)
            if desired_vel != 0 and angle_difference < (5 * math.pi/180):
                for _ in range(ultra_pings_loop):
                    raw_distances = get_all_distance_looped()
                    add_ultras_to_grid(raw_distances, current['x'], current['y'], current['angle'], obstacles)

            # Pruning (Don't need to do every loop)		
            obstacles = prune_old_obstacles(obstacles, obstacle_lifetime)
            prune_old_grid_entries(grid_memory_time)
			#################################################

			## ------------ Update waypoints ----------- ##
            check = check_front()
            if check:
				# Make a new list of just the main waypoints
                new_waypoints = []
                for a_waypoint in waypoints:
                    if a_waypoint[3]:
                        new_waypoints.append(a_waypoint)
						
				# Add the interwaypoints to the front of the new waypoints list
                waypoints = check + new_waypoints

                # Set next waypoint    
                goal_x, goal_y, goal_angle, is_main = waypoints[0]

				# Update
                distance_to_waypoint = math.sqrt((goal_x - current['x'])**2 + (goal_y - current['y'])**2)
                angle_difference = correct_angle(goal_angle - current['angle'])

				# Reset grid and obstacles so we don't pile up
                obstacles = []
                grid = grid_template[:]
                grid_timestamps = grid_timestamps_template[:]
			#################################################
            
            if collision_check and (200 < current['y'] < 600):
                distance_2 = get_distance(sensors['sensor_2']['trigger'], sensors['sensor_2']['echo'])
                print(f"Checking in front. Distance 2 is {distance_2}")
                if distance_2 and (distance_2 < collision_forsight):
                    collision_count += 1
                    print("Count incremented")
                    if collision_count >= collision_limit:
                        return 'go home'

            ## ------- Printing/Debug -------------- ##
            s = "Pos({:<6.1f} {:<6.1f} {:<6.1f}°) | Loop {:<6.3f}< {:<6.3f}s| D2WP {:<6.1f}| Vel (L: {}<=>{}, R: {}<=>{}) | DC (L: {}, R: {}) | T (T-Vel: {:<6.1f}, An-Vel: {:<6.1f})"
            print(s.format(current['x'], current['y'], (current['angle'] * 180/math.pi), time_since_loop_tentacles, control['loop_sleep'], distance_to_waypoint, color(vel['left']), color(desired_vel_left), color(vel['right']), color(desired_vel_right), color(duty_cycle['left']), color(duty_cycle['right']), desired_vel, desired_angular_vel)) if debug else None
		
    # Clean up if Interrupt
    except KeyboardInterrupt:
        cleanup(True)
        

if __name__ == "__main__":
    move(waypoints,True,False)
