"""
Servo movement example in "joint" mode

This script demonstrates the use of the armlab_xl320_library to control servo motors.
The servo is set to operate in "joint" mode, where it alternates between two goal positions we set.

Use: sudo python3 rotate_full_range.py
"""

from armlab_xl320_library import *

CONNECTION_DEVICE = "USB" 
PORT_NAME = "/dev/ttyACM0"  # USB port names are dynamic you need to check what it is 

# User defined value
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

def main():
    if CONNECTION_DEVICE == "USB":
        portHandler, packetHandler = initialize_handlers(PORT_NAME)
    else:
        print("Invalid connnection device!")

    # defines the servo's ID
    servo1_ID = 10 # top servo
    servo2_ID = 11 # bottom servo

    open_port(portHandler)
    set_baudrate(portHandler, 1000000)

    # Initialize a Servo instance
    servo1 = Servo(servo1_ID, portHandler, packetHandler)
    servo1.change_led_color(LED_PURPLE)
    servo1.disable_torque()
    
    # Sets servo1 to joint mode
    servo1.set_control_mode("joint")  # torque must be off when you change mode
    servo1.enable_torque()

    # Initialize second Servo instance
    servo2 = Servo(servo2_ID, portHandler, packetHandler)
    servo2.change_led_color(LED_YELLOW)
    servo2.disable_torque()

    # Sets servo2 to joint mode
    servo2.set_control_mode("joint")  # torque must be off when you change mode
    servo2.enable_torque()

    index_s1 = 1

    # raw encoder unit - convert to deg = (deg val/360)x4095 rad = (val/2pi)x4095
    # Servo 1 (top servo) - should rotate between 0 to 40 deg (35 deg for tolerance)
    goal_positions_s1 = [
        480, # in raw encoder unit
        100,
        570, # 20 deg rotation
        #116, # 40 deg rotation
    ]
    
    index_s2 = 1
    
    # Servo 2 (bottom servo) - should rotate between 0 to 90 deg (80 deg for tolerance)
    goal_positions_s2 = [
        0, # in raw encoder unit
        910, # 35 deg rotation
    ]
    servo1.set_joint_speed(300)  # range(0,1023)
    servo2.set_joint_speed(300)  # range(0,1023)

    while True:
        print("Press Enter to go to Goal Position")
        if getch() == chr(0x1B): # Detects if Escape Key was Pressed if it is, the code terminates
            index_s1 = 0
            index_s2 = 0
            servo1_current_position = servo1.get_position()
            servo1.set_position(goal_positions_s1[index_s1])
            servo2.set_position(goal_positions_s2[index_s2])
            print("[ID:%d] GoalPos:%d  PresPos:%d"  % (servo1_ID, goal_positions_s1[index_s1], servo1_current_position))
            print("[ID:%d] GoalPos:%d  PresPos:%d"  % (servo2_ID, goal_positions_s2[index_s2], servo2_current_position))
            break
        
        servo1.set_position(goal_positions_s1[index_s1])
        servo2.set_position(goal_positions_s2[index_s2])
        while True:
            servo1_current_position = servo1.get_position()
            servo2_current_position = servo2.get_position()

            if goal_positions_s1[index_s1] is None:
                continue
            
            print("---")
            print("[ID:%d] GoalPos:%d  PresPos:%d"  % (servo1_ID, goal_positions_s1[index_s1], servo1_current_position))
            print("[ID:%d] GoalPos:%d  PresPos:%d"  % (servo2_ID, goal_positions_s2[index_s2], servo2_current_position))

            if (not abs(goal_positions_s1[index_s1] - servo1_current_position)> DXL_MOVING_STATUS_THRESHOLD) \
                and (not abs(goal_positions_s2[index_s2] - servo2_current_position)> DXL_MOVING_STATUS_THRESHOLD):
                    print("stopping")
                    break
        
        #break
       # Change goal position
        if index_s1 == 1:
            index_s1 = 2
        else:
            index_s1 = 1
            
        if index_s2 == 0:
            index_s2 = 1
        else:
            index_s2 = 0
        
    close_port(portHandler)

if __name__ == "__main__":
    main()
