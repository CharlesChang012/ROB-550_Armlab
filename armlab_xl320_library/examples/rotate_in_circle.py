"""
Servo movement example in "wheel" mode

This script demonstrates the use of the armlab_xl320_library to control servo motors
operating in wheel mode. The servo rotates continuously with a user-defined speed until 
a stop command is issued.

Use: sudo python3 rotate_in_circle.py
"""


# import sys
# sys.path.append('~/armlab_xl320_library/src/armlab_xl320_library')

from armlab_xl320_library import *

CONNECTION_DEVICE = "USB" 
PORT_NAME = "/dev/ttyACM0"  # USB port names are dynamic you need to check what it is 

def main():
    if CONNECTION_DEVICE == "USB":
        portHandler, packetHandler = initialize_handlers(PORT_NAME)
    else:
        print("Invalid connnection device!")

    # defines the servo's ID
    servo1_ID = 9
    #servo2_ID = 6

    open_port(portHandler)
    set_baudrate(portHandler, 1000000)

    servo1 = Servo(servo1_ID, portHandler, packetHandler)
    servo1.change_led_color(LED_CYAN)
    servo1.disable_torque()
    servo1.set_control_mode("wheel")  # torque must be off when you change mode
    servo1.enable_torque()

    #servo2 = Servo(servo2_ID, portHandler, packetHandler)
    #servo2.change_led_color(LED_PURPLE)
    #servo2.disable_torque()
    #servo2.set_control_mode("wheel")  # torque must be off when you change mode
    #servo2.enable_torque()

    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1B):
        close_port(portHandler)
        quit()

    servo1.set_wheel_cw_speed(50)  # value is in percentage
    #servo2.set_wheel_ccw_speed(30)  # value is in percentage

    try:
        print("Press ESC to stop the servo.")
        while True:  # Start a loop that will run until ESC is pressed
            if getch() == chr(0x1B):
                servo1.set_wheel_cw_speed(0)  # Stop the servo by setting speed to 0
                #servo2.set_wheel_ccw_speed(0)  # Stop the servo by setting speed to 0
                break

    except KeyboardInterrupt:
        servo1.set_wheel_cw_speed(0)
        #servo2.set_wheel_ccw_speed(0)

    finally:
        servo1.disable_torque()
        #servo2.disable_torque()
        close_port(portHandler)

if __name__ == "__main__":
    main()
