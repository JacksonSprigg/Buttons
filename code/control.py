# Controller for ECE4191 robot
import time
import math
import RPi.GPIO as GPIO

# Camera imports
import cv2
from pyzbar.pyzbar import decode
from picamera2 import MappedArray, Picamera2, Preview

# Other files
from movement_v8_17_oct import move, cleanup, current, get_all_distance_looped, color, enc_count_conveyer, motorconveyer_PWM1, motorconveyer_PWM2, calibrate, button_1, button_3, button_4, button_6

'''
Acts as the main state machine, importing required functionality from movement_and_sensors.py
'''

######### PARAMETER TIME ##############

## ------- Camera params ----------##
colour = (0, 255, 0)
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 1
thickness = 2
picam2 = Picamera2()
barcodes = []
picam2.start()

## ---------  States --------- ##
LOAD_STATE     = 'loading'
MOVE_STATE     = 'moving'
DELIVERY_STATE = 'delivering'
RETURN_STATE   = 'returning'

# Wait time
DEPART_WAIT    = 1     

# Wall avoidance
offset_y = 220
offset_x = 200
GOAL_LOCATIONS = {
    'A':(offset_x,  (1200 - offset_y), math.radians(90), True), 
    'B':(600,  (1200 - offset_y), math.radians(90), True),
    'C':((1200 - offset_x+10), (1200 - offset_y), math.radians(90), True)
    }                                   # CONFIRMATION NEEDED

# Thresholding
DIST_THRESHOLD = 20                     
ANGLE_THRESHOLD = math.radians(3)        

# Length of belt in cm
LEN_BELT = 30 

# Shaft rotation
conveyer_roll_diam      = 30
gear_ratio              = 20.4
circum                  = math.pi * conveyer_roll_diam
counts_per_rev_conveyor = 12 * gear_ratio
dist_per_count_conveyer = (circum / counts_per_rev_conveyor)

run_data = {
    'goals': [],
    'current_location': 0,       # 0 = Loading | 1 = goal A | 2 = goal B | 3 = goal C
    'waypoints': [],
    'belt_enc_count': 0,
    'conveyer_lengths':{
        'A': 0,
        'B': 0,
        'C': 0
    } 
}

##---------- State Machine ------------##
# Set initial state define state machine
state = LOAD_STATE
def main():
    global state
    try:
        while True:
            if state == LOAD_STATE:
                loading_state()
                state = MOVE_STATE
                continue
            elif state == MOVE_STATE:
                flag_home = moving_state()
                print(f"flag is {flag_home}")
                if flag_home == "go home":
                    state = RETURN_STATE
                    run_data['goals'] = []
                else:
                    state = DELIVERY_STATE
            elif state == DELIVERY_STATE:
                delivery_state()
                if len(run_data['goals']) != 0:
                    state = MOVE_STATE
                else:
                    state = RETURN_STATE
            elif state == RETURN_STATE:
                return_state()
                state = LOAD_STATE
            else:
                raise Exception('Undefined state encountered')
            
    except KeyboardInterrupt:
        print("Exiting")
        cleanup(False)

##--------- State functions ---------- ##
def loading_state():
    print('\n~Loading state~')
    print('ACTION REQUIRED: Please present QR code')

    # Turn buttons into outputs for flashing
    for pin in [button_1, button_3]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)

    # Sit in loop looking at camera every 0.2s
    camera_read_output = None
    flash_counter = 0
    flash_every_x_loopsleep = 4

    while camera_read_output is None:
        time.sleep(0.05)
        camera_read_output = camera_read()
        #### Flash #######################
        if (flash_counter % flash_every_x_loopsleep == 0) and (flash_counter % (flash_every_x_loopsleep * 2) != 0):
            GPIO.output(button_1, 1)
            GPIO.output(button_3, 0)
                
        elif flash_counter % (flash_every_x_loopsleep * 2) == 0:
            GPIO.output(button_1, 0)
            GPIO.output(button_3, 1)

        flash_counter += 1
        ##############################

    # Reset make all pins inputs again
    for pin in [button_4, button_6, button_1, button_3]:
        GPIO.setup(pin, GPIO.IN)
    invert = False
    # Set data
    if camera_read_output == 'A':
        camera_data = [LEN_BELT,0,0]
        print("-> Objective set to A")
    elif camera_read_output == 'B':
        camera_data = [0,LEN_BELT,0]
        print("-> Objective set to B")
    elif camera_read_output == 'C':
        camera_data = [0,0,LEN_BELT]
        print("-> Objective set to C")
    elif camera_read_output == 'AB':
        camera_data = [LEN_BELT/2,LEN_BELT/2,0]
        print("-> Objective set to A and B")
    elif camera_read_output == 'AC':
        camera_data = [LEN_BELT/2,0,LEN_BELT/2]
        print("-> Objective set to A and C")
    elif camera_read_output == 'BC':
        camera_data = [0,LEN_BELT/2,LEN_BELT/2]
        print("-> Objective set to B and C")
    elif camera_read_output == 'BA':
        camera_data = [LEN_BELT/2,LEN_BELT/2,0]
        invert = True
        print("-> Objective set to B and A")
    elif camera_read_output == 'CA':
        camera_data = [LEN_BELT/2,0,LEN_BELT/2]
        invert = True
        print("-> Objective set to C and A")
    elif camera_read_output == 'CB':
        camera_data = [0,LEN_BELT/2,LEN_BELT/2]
        invert = True
        print("-> Objective set to C and B")
    else:
        print("-> False QR data")

    # Define data
    A_length = camera_data[0]
    B_length = camera_data[1] 
    C_length = camera_data[2]

    # Set deposit lengths
    run_data['conveyer_lengths'] = {
        'A': A_length,
        'B': B_length,
        'C': C_length
    } 
    
    # Set goals
    if invert == False:
        if A_length > 0:
            run_data['goals'].append('A')   
        if B_length > 0:
            run_data['goals'].append('B')      
        if C_length > 0:
            run_data['goals'].append('C')       
    elif invert == True:
        if C_length > 0:
            run_data['goals'].append('C')   
        if B_length > 0:
            run_data['goals'].append('B')      
        if A_length > 0:
            run_data['goals'].append('A')       

def moving_state():
    print('\n~Moving state~')
    
    # Set the first waypoint from goals
    goal = run_data['goals'][0]

    # Get away from the y axis walls before we start moving (Less than or equal to because we start at 150)
    if current['y'] > 1050:
        intermediate_waypoint = (current['x'], (1200 - offset_y), math.radians(90), True)
        waypoints = [intermediate_waypoint, GOAL_LOCATIONS[goal]]
    elif current['y'] <= 150:
        intermediate_waypoint = (current['x'], offset_y, math.radians(90), True)
        waypoints = [intermediate_waypoint, GOAL_LOCATIONS[goal]]
    else:
        waypoints = [ GOAL_LOCATIONS[goal] ]
    
    print(f"-> Intermediate set to: ({intermediate_waypoint[0]:.1f}, {intermediate_waypoint[1]:.1f}, {intermediate_waypoint[2] * (190/math.pi):.1f})")

    # Start moving
    flag_home = move(waypoints, True, False)
    return flag_home




def delivery_state():
    print('\n~Delivery state~')
    
    # Set goal
    goal = run_data['goals'][0]   
    
    # Localise against walls
    calibrate('delivering')
    
    print(f"-> Arrived at goal {goal}")

    # Move conveyor belt either half or full
    length = run_data['conveyer_lengths'][goal]
    conveyor_forward(length)

    # Delete most recent goal
    del run_data['goals'][0]


def return_state():
    print('\n~Return state~')

    # Add loading zone to waypoints list
    waypoints = [(949, offset_y, math.radians(90), True)]

    # Get away from the y axis walls before we start moving (Less than or equal to because we start at 150)
    if current['y'] > 1050:
        intermediate_waypoint = (current['x'], 900, math.radians(90), True)
        waypoints = [intermediate_waypoint] + waypoints
    elif current['y'] <= 150:
        intermediate_waypoint = (current['x'], 300, math.radians(90), True)
        waypoints = [intermediate_waypoint] + waypoints
    

    # Move
    move(waypoints, True, False)

    # Localise against walls
    calibrate('loading')


def camera_read():
    '''
    The camera read function will take an image from the camera, process it to find 
    INPUTS
    -
    OUTPUTS
    - list, if valid, with elements [goal1, goal2, goal3, goal4, x_correction, y_correction, angle_correction]
    - list, if valid, with elements [cutoff_AB, cutoff_BC, angle offset]
    - 0 = goalA, 1 = goalB, 2 = goalC, -1 = no parcel
    - None, if invalid
    '''
    
    rgb = picam2.capture_array("main")
    barcodes = decode(rgb)
    if barcodes:
        data = barcodes[0].data.decode()
        return data
    else:
        return None


def conveyor_forward(length):
    '''
        conveyor_forward(n) function
        Moves the conveyor belt fowards by 'length' cm. The belt is broken up into  increments.
        INPUTS
        length: length to move conveyer belt, in cm
    '''
    length = 10*length      # convert cm to mm
    enc_count_limit = (enc_count_conveyer[0]) + length/dist_per_count_conveyer        # needs to be different dist_per_count

    while (enc_count_conveyer[0]) < enc_count_limit:                       # only counts up in negative
        motorconveyer_PWM1.ChangeDutyCycle(4)
        motorconveyer_PWM2.ChangeDutyCycle(0)
        time.sleep(0.05)

    motorconveyer_PWM1.ChangeDutyCycle(0)
    motorconveyer_PWM2.ChangeDutyCycle(0)    

# Testing interface before opertions begin
if __name__ == "__main__":
    while True:
        choices = ["Normal operation", 'Ultrasonic testing', 'Button testing','Camera testing', 'Barcode testing']
        print("\033c")
        print("==== MENU ====")
        for i in range(len(choices)):
            print(f"({i}): {choices[i]}")
        choice = int(input("Enter a choice: "))
        if choice == 0:
            main()
            print("Not allowed to return to main menu after normal operation because too hard to program. Exiting completely...")
            break
        elif choice == 1:
            try:
                while True:
                    time.sleep(0.5)

                    us_list = get_all_distance_looped()
                    print_list = []
                    for i in range(6):
                        if us_list[i]:
                            print_list.append(color(i+1, _color='green_digit'))
                            us_list[i] = round(us_list[i])
                        else:
                            print_list.append(color(i+1, _color='red_digit'))
                            us_list[i] = 'None'
                    print("\033c")
                    print(f"""
            {us_list[1]:4}   
{us_list[0]:4} ___________________ {us_list[2]:4}
    | {print_list[0]}       {print_list[1]}       {print_list[2]} |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |_{print_list[5]}_______{print_list[4]}_______{print_list[3]}_|
{us_list[5]:4}                     {us_list[3]:4}
            {us_list[4]:4}
                    """)
                    
                    if not ('None' in us_list):
                        print(f"Front sum: {us_list[0]+us_list[2]} and Back sum: {us_list[3]+us_list[5]}.")
            except KeyboardInterrupt:
                pass
        elif choice == 2:
            try:
                while True:
                    time.sleep(0.2)
                    # Check what buttons are doing
                    labels = [1,3,4,6]
                    button_data = [GPIO.input(button_1), GPIO.input(button_3), GPIO.input(button_4), GPIO.input(button_6)]
                    print_list = []
                    for i in range(len(button_data)):
                        if button_data[i]:
                            print_list.append(color(labels[i], _color='green_digit'))
                        else:
                            print_list.append(color(labels[i], _color='red_digit'))
                    print("\033c")
                    print(f'''
     ___________________
    | {print_list[0]}               {print_list[1]} |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |_{print_list[3]}_______________{print_list[2]}_|
''')
                    
            except KeyboardInterrupt:
                pass    
            
        elif choice == 3:
            try:
                while True:
                    # i+=1
                    picam2.capture_file(f"image_testing.jpg")
                    time.sleep(0.2)
                    print('Image captured')
            except KeyboardInterrupt:
                pass
        elif choice == 4:
            try:
                while True:
                    time.sleep(0.05)
                    while True:
                        rgb = picam2.capture_array("main")
                        barcodes = decode(rgb)
                        # print('Checking')
                        if barcodes:
                            code = barcodes[0].data.decode()
                            print(f'Found {code}')
                            time.sleep(0.5)
            except KeyboardInterrupt:
                pass

"""
[23,19,20,None,12,None]

becomes

     ___________________
    | 1       2       3 |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |                   |
    |_6_______5_______4_|


"""
