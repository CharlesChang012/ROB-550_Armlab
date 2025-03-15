"""!
The state machine that implements the logic.
"""
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
import time
import numpy as np
import rclpy

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,          0.0,        0.0],
            [0.75*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.5*-np.pi/2,   -0.5,      -0.3,      np.pi/2,        0.0],
            [0.25*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.0,             0.0,       0.0,          0.0,        0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [0.5*np.pi/2,     0.5,       0.3,     -np.pi/3,        0.0],
            [0.75*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [np.pi/2,         0.5,       0.3,     -np.pi/3,        0.0],
            [0.0,             0.0,       0.0,          0.0,        0.0]]
        
        self.current_t = 0
        self.teach_repeat_waypoints = []
        self.teach_repeat_open_idx = []
        self.teach_repeat_close_idx = []

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and functions as needed.
        """

        # IMPORTANT: This function runs in a loop. If you make a new state, it will be run every iteration.
        #            The function (and the state functions within) will continuously be called until the state changes.

        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "teach":
            self.teach()

        if self.next_state == "repeat":
            self.repeat()

    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"

        for i in range(np.size(self.waypoints, 0)):
            self.rxarm.set_positions(self.waypoints[i])
            time.sleep(1)

        self.next_state = "idle"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        
        # Extrinsic Matrix Calibration
        K = self.camera.intrinsic_matrix
        D = self.camera.distortion_matrix

        #world_points = [[-250, -25, 0],[250, -25, 0],[250, 275, 0],[-250, 275, 0], [350, -25, 183], [-350, 125, 98]]
        world_points = [[-250, -25, 0],[250, -25, 0],[250, 275, 0],[-250, 275, 0]]
        img_points = self.camera.apriltags_centers
        
        
        H = self.camera.recover_homogenous_transform_pnp(img_points, world_points, K, D)
        #H = self.camera.recover_homogeneous_transform_svd_modified(img_points, world_points)
        print(H)

        self.camera.H_matrix = H
        self.camera.K_matrix = K

        # Homography Calibration
        self.camera.homography_transform()
        self.camera.camera_calibrated = True 

        #print(self.camera.H_matrix)

        self.status_message = "Calibration - Completed Calibration"
        

    
    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        time.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            time.sleep(5)
        self.next_state = "idle"

    def teach(self):
        sample_time = 60
        sample_period = 0.5
        self.current_state = "teaching"
        self.status_message = "Teaching Started"
        print("Starting Recording")

        self.teach_repeat_waypoints= np.zeros((int(sample_time/sample_period),5))
        self.teach_repeat_open_idx = []
        self.teach_repeat_close_idx = []
        
        for t in range(int(sample_time/sample_period)):
            self.teach_repeat_waypoints[t] = self.rxarm.get_positions()
            self.current_t = t
            print(f"Shape of waypoints:{np.shape(self.teach_repeat_waypoints)}")
            print(f"Current Points:{self.rxarm.get_positions()}")
            print(f"Last Recorded:{self.teach_repeat_waypoints[t]}")
            time.sleep(sample_period)
        
        with open('angles.txt', 'w') as f:
            for row in self.teach_repeat_waypoints:
                f.write(f"{row[0]},{row[1]},{row[2]},{row[3]},{row[4]}")
                f.write("\n")
            f.close()

 
        print("Stopping Recording")
        self.next_state = "idle"

    def repeat(self):
        self.current_state = "repeating"
        self.status_message = "Repeating Started"
        for i in range(np.size(self.teach_repeat_waypoints, 0)):
            self.rxarm.set_positions(self.teach_repeat_waypoints[i])
            if i in self.teach_repeat_close_idx:
                self.rxarm.gripper.grasp()
            if i in self.teach_repeat_open_idx:
                self.rxarm.gripper.release()
            time.sleep(1)
        self.next_state = "idle"


class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            time.sleep(0.05)
