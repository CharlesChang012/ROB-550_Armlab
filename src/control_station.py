#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os, sys
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import cv2
import numpy as np
import rclpy
import time
from functools import partial

from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QPixmap, QImage, QCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMainWindow, QFileDialog

from resource.ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread

import subprocess
import multiprocessing


""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]

        # task 1.3 - First attempt not working
        #self.record_flag = False
        #self.record_process = ''
        """User defined variables"""
        self.pt_world_frame = [0,0,0]
        self.click_to_grab_flag = True
        self.click_to_place_flag = False
        
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())
        self.ui.btn_calibrate.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))

        # User Buttons
        # TODO: Add more lines here to add more buttons
        # To make a button activate a state, copy the lines for btnUser3 but change 'execute' to whichever state you want
        self.ui.btnUser1.setText('Open Gripper')
        self.ui.btnUser1.clicked.connect(lambda: self.rxarm.gripper.release())
        self.ui.btnUser2.setText('Close Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.rxarm.gripper.grasp())
        self.ui.btnUser3.setText('Execute')
        self.ui.btnUser3.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        
        self.ui.btnUser4.setText('inv_kin') # User defined
        self.ui.btnUser4.clicked.connect(self.find_kinematics)   # User defined

        # task 1.3 - Current
        self.ui.btnUser5.setText('Teach') # User defined
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'teach'))   # User defined
        self.ui.btnUser6.setText('Repeat') # User defined
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'repeat'))   # User defined
        self.ui.btnUser7.setText('Teach Open Gripper') # User defined
        self.ui.btnUser7.clicked.connect(self.teach_open)   # User defined 
        self.ui.btnUser8.setText('Teach Close Gripper') # User defined
        self.ui.btnUser8.clicked.connect(self.teach_close)   # User defined 
        self.ui.btnUser9.setText('Competition 1') # User defined
        self.ui.btnUser9.clicked.connect(self.competition1)   # User defined 
        self.ui.btnUser10.setText('Competition 2') # User defined
        self.ui.btnUser10.clicked.connect(self.competition2)   # User defined 
        self.ui.btnUser11.setText('Competition 3') # User defined
        self.ui.btnUser11.clicked.connect(self.competition3)   # User defined 
        self.ui.btnUser12.setText('Competition 4') # User defined
        self.ui.btnUser12.clicked.connect(self.competition4)   # User defined 

        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    # Distances should be in mm
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f rad" % (pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f rad" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, grid_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(grid_image))

    """ Other callback functions attached to GUI elements"""


    # task 1.3 - Currrent
    def teach_open(self):
        print("Teaching Open")
        self.sm.teach_repeat_open_idx.append(self.sm.current_t)
        print(f"Array Size:{np.size(self.sm.teach_repeat_waypoints, axis=0)}")
        print(f"IDX of Open:{self.sm.current_t}")

    def teach_close(self):
        print("Teaching Close")
        self.sm.teach_repeat_close_idx.append(self.sm.current_t)

        print(f"Array Size:{np.size(self.sm.teach_repeat_waypoints, axis=0)}")
        print(f"IDX of Close:{self.sm.current_t}")
    

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')
    
    def find_kinematics(self):
        self.rxarm.test_inv_kin()

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        self.camera.loadCameraCalibration('')

        # TODO: Modify this function to change the mouseover text.
        # You should make the mouseover text display the (x, y, z) coordinates of the pixel being hovered over

        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            
            if self.camera.camera_calibrated:
                z=self.camera.DepthFrameWarped[pt.y()][pt.x()]
            else:
                z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))
            
            if self.camera.camera_calibrated:
                # Perform homography transformation on mouse coord in warp img frame back to original img frame then to world frame
                pt_img_frame = np.linalg.inv(self.camera.homography_matrix) @ np.array([[pt.x()], [pt.y()], [1]])
                lamda = pt_img_frame[2]
                self.pt_world_frame = self.camera.pixel2world_coord([pt_img_frame[0]/lamda, pt_img_frame[1]/lamda, self.camera.DepthFrameRaw[int(pt_img_frame[1]/lamda)][int(pt_img_frame[0]/lamda)]])
                #self.pt_world_frame = self.camera.pixel2world_coord([pt.x(), pt.y(), z])
            else:
                self.pt_world_frame = self.camera.pixel2world_coord([pt.x(), pt.y(), z])
            
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" %
                                             (self.pt_world_frame[0], self.pt_world_frame[1], self.pt_world_frame[2]))

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True

        ## TODO: Implement click to pick / click to drop
        if self.camera.camera_calibrated == True:
            self.rxarm.target_pos = self.pt_world_frame
            if self.click_to_grab_flag == True:
                target_idx = np.argmin(np.linalg.norm(self.rxarm.target_pos  - self.camera.block_detection.block_world_coords))
                self.rxarm.target_orient = self.camera.block_detection.block_orients[target_idx]
                self.rxarm.click_to_grab()
                self.click_to_grab_flag = False
                self.click_to_place_flag = True
            elif self.click_to_place_flag == True:
                self.rxarm.click_to_place()
                self.click_to_place_flag = False
                self.click_to_grab_flag = True

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


    def deStack(self, idx, destack_idx):
        self.rxarm.target_pos = self.camera.block_detection.block_world_coords[idx]
        self.rxarm.target_orient = self.camera.block_detection.block_orients[idx]
        self.rxarm.click_to_grab()
        self.rxarm.target_pos = self.rxarm.destack_des[destack_idx]
        #self.rxarm.target_pos[2] = self.rxarm.target_pos[2] + 60
        self.rxarm.target_orient = 0
        self.rxarm.click_to_place()
    
    def competition1(self):
        large_dest = [[200, -75, 5], [200, -75, 40], [200, -75, 65]]
        small_dest = [[-200, -75, 3], [-200, -70, 18], [-200, -70, 37]]
        destack_idx = 0
        self.rxarm.set_positions([0, 0, -np.pi/2, 0, 0], True)
        time.sleep(2)
        while not next((i for i, position in enumerate(self.camera.block_detection.block_world_coords) if position[2] > 45), None) == None:
            print("destacking")
            print("z value:")
            print(self.camera.block_detection.block_world_coords[2])
            idx = next((i for i, position in enumerate(self.camera.block_detection.block_world_coords) if position[2] > 45), None)
            destack_idx += 1
            self.deStack(idx, destack_idx)
        self.camera.block_detection.sort_key = "color"
        time.sleep(3)
        self.rxarm.target_size = None
        self.rxarm.block_world_coords = self.camera.block_detection.block_world_coords
        self.rxarm.block_orients = self.camera.block_detection.block_orients
        self.rxarm.block_uvd = self.camera.block_detection.block_uvd

        for i in range(6):
            #print("index:", i)
            #print(self.rxarm.block_uvd)
            #print(self.rxarm.block_world_coords[i])
            #print(self.rxarm.block_orients[i])
            self.rxarm.target_pos = self.rxarm.block_world_coords[i]
            self.rxarm.target_orient = self.rxarm.block_orients[i]
            self.rxarm.click_to_grab()
            # grabing large first
            if i < 3:
                self.rxarm.target_pos = large_dest[i]
                self.rxarm.target_orient = 0
                self.rxarm.target_size = 'large'
                self.rxarm.click_to_place()
                #large_dest[2] += 35
            else:
                self.rxarm.target_pos = small_dest[i-3]
                self.rxarm.target_orient = 0
                self.rxarm.target_size = 'small'
                self.rxarm.click_to_place()
                #small_dest[2] += 20

    def competition2(self):
        large_dest = [250, -50, 0]
        small_dest = [-250, -50, 0]
        destack_idx = 0
        self.rxarm.set_positions([0, 0, -np.pi/2, 0, 0], True)
        time.sleep(2)
        self.camera.only_blocks = False
        while not next((i for i, position in enumerate(self.camera.block_detection.block_world_coords) if position[2] > 45), None) == None:
            print("destacking")
            print("z value:")
            print(self.camera.block_detection.block_world_coords[2])
            idx = next((i for i, position in enumerate(self.camera.block_detection.block_world_coords) if position[2] > 45), None)
            destack_idx += 1
            self.deStack(idx, destack_idx)
        self.camera.only_blocks = True
        self.camera.block_detection.sort_key = "color"
        time.sleep(3) 
        self.rxarm.target_size = None
        self.rxarm.block_world_coords = self.camera.block_detection.block_world_coords
        self.rxarm.block_orients = self.camera.block_detection.block_orients
        self.rxarm.block_uvd = self.camera.block_detection.block_uvd
        #print(self.camera.block_detection.large_num)
        
        for i in range(12):
            #print(self.rxarm.block_uvd[i])
            #print(self.rxarm.block_world_coords[i])
            #print(self.rxarm.block_orients[i])
            self.rxarm.target_pos = self.rxarm.block_world_coords[i]
            self.rxarm.target_orient = 0#self.rxarm.block_orients[i]
            self.rxarm.click_to_grab()
            # grabing large first
            if i < 6:
                self.rxarm.target_pos = large_dest
                self.rxarm.target_orient = 0
                self.rxarm.target_size = 'large'
                self.rxarm.click_to_place()
                large_dest[1] += 60
            else:
                self.rxarm.target_pos = small_dest
                self.rxarm.target_orient = 0
                self.rxarm.target_size = 'small'
                self.rxarm.click_to_place()
                small_dest[1] += 40

    def competition3(self):
        dest = [0, 300, 0]
        start_time = time.time()
        block_cnt = 1
        self.rxarm.set_positions([0, 0, -np.pi/2, 0, 0], True)
        self.camera.block_detection.sort_key = "dist"
        time.sleep(2)
        #self.rxarm.block_world_coords = self.camera.block_detection.block_world_coords
        #self.rxarm.block_orients = self.camera.block_detection.block_orients
        #self.rxarm.block_uvd = self.camera.block_detection.block_uvd
        while time.time() - start_time < 600:
            idx = 0
            while True:
                target_world_coord = self.camera.block_detection.block_world_coords[idx]
                tx = target_world_coord[0]
                ty = target_world_coord[1]
                if -30 < tx < 30 and ty >= 300:
                    idx += 1
                else:
                    break
            print(idx)
            self.rxarm.target_pos = self.camera.block_detection.block_world_coords[idx]
            self.rxarm.target_orient = self.camera.block_detection.block_orients[idx]
            self.rxarm.click_to_grab()
            self.rxarm.target_pos = dest
            self.rxarm.target_orient = 0
            if block_cnt > 2:
                psi = np.pi/2
            elif block_cnt <= 2:
                psi = np.pi
            if block_cnt == 3:
                dest[1] += 10
                dest[2] += 25
            if block_cnt == 4:
                dest[2] -= 10
            self.rxarm.click_to_place(psi)
            dest[2] += 35
            block_cnt += 1
    
    def competition4(self):
        self.camera.depth_high_thresh = 1005
        self.camera.only_blocks = False
        time.sleep(2)

        
        load_position =   [0,300,100]
        while(True):
            if np.array(self.camera.block_detection.block_world_coords).any():
                c = self.camera.block_detection.block_world_coords[0]
                self.rxarm.target_pos = c
                self.rxarm.target_orient = 0
                self.rxarm.click_to_grab()
                self.rxarm.set_positions([0, 0, -np.pi/2, 0, 0], True)
                self.rxarm.target_pos = load_position
                self.rxarm.target_orient = 0
                #self.rxarm.set_positions([np.deg2rad(-64.95), np.deg2rad(16.7), np.deg2rad(-13.01), np.deg2rad(68.29), np.deg2rad(5)], True)
                self.rxarm.set_positions([np.deg2rad(-63.72), np.deg2rad(7.91), np.deg2rad(-28.04), np.deg2rad(54.76), np.deg2rad(-0.53)], True)
                self.rxarm.gripper.release()
                self.rxarm.set_positions([0, 0, -np.pi/2, 0, 0], True)

            time.sleep(2)


### TODO: Add ability to parse POX config file as well
def main():
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui()
    app_window.show()

    # Set thread priorities
    app_window.VideoThread.setPriority(QThread.HighPriority)
    app_window.ArmThread.setPriority(QThread.NormalPriority)
    app_window.StateMachineThread.setPriority(QThread.LowPriority)

    sys.exit(app.exec_())


# Run main if this file is being run directly
if __name__ == '__main__':
    main()
