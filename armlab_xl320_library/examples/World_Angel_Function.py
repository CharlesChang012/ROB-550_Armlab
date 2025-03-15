"""
Servo movement example in "joint" mode

This script demonstrates the use of the armlab_xl320_library to control servo motors.
The servo is set to operate in "joint" mode, where it alternates between two goal positions we set.

Use: sudo python3 rotate_full_range.py
"""

from armlab_xl320_library import *
import sys  # For system exit

# Connection settings
CONNECTION_DEVICE = "USB"
PORT_NAME = "/dev/ttyACM2"  # USB port names are dynamic, check before running
SERVO_ID = 10  # Define the servo ID

# User-defined values
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

def world_angle(cannon_angle):
    """Convert cannon_angle to servo world angle."""
    offset = 58.06
    return offset - cannon_angle

def move2angle(cur_angle,PORT_NAME):
    """Move servo to the specified angle."""
    bitval = int((cur_angle / 360) * 4095)  # Convert angle to bit value

    if CONNECTION_DEVICE == "USB":
        portHandler, packetHandler = initialize_handlers(PORT_NAME)
    else:
        print("Invalid connnection device!")

    # Initialize servo
    servo = Servo(SERVO_ID, portHandler, packetHandler)
    servo.change_led_color(LED_PURPLE)
    servo.disable_torque()
    servo.set_control_mode("joint")  # Torque must be off when changing mode
    servo.enable_torque()
    servo.set_joint_speed(300)  # Speed range(0, 1023)

    # Move servo
    servo_current_position = servo.get_position()
    servo.set_position(bitval)
    print(f"---\n[ID:{SERVO_ID}] GoalPos:{bitval}  PresPos:{servo_current_position}")

    # Wait for user input to terminate
    try:
        if getch() == chr(0x1B):  # Escape key pressed
            servo_current_position = servo.get_position()
            servo.set_position(500)  # Reset position
            print(f"[ID:{SERVO_ID}] GoalPos:500  PresPos:{servo_current_position}")
    except:
        pass

    # Close port
    close_port(portHandler)

def main():
    """Main function to get user input and move servo accordingly."""
    while True:
        user_in = input("Enter Angle (or 'q' to quit): ")
        
        try:
            cannon_angle = float(user_in)  # Convert input to float
            cur_angle = world_angle(cannon_angle)
            move2angle(cur_angle,PORT_NAME)
        except ValueError:
            print("Invalid input! Please enter a valid number.")

        # if user_in.lower() == 'q':
        #     print("Exiting...")
        #     sys.exit()


if __name__ == "__main__":
    main()
