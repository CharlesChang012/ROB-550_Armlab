#!/usr/bin/env python3

"""!
Class to represent the camera.
"""
 
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import cv2
import time
import numpy as np
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

class BlockDetection():
    def __init__(self):
        """User defined parameters for block detecdtion"""
        self.block_num = 0
        self.large_num = 0
        self.block_contours = None
        self.block_uvd = None
        self.block_world_coords = None
        self.block_dist = None
        self.block_orients = None
        self.block_sizes = None    # 0 for large, 1 for small
        self.block_color_idxs = None
        self.sort_key = "color"

    def update(self):
        self.block_contours = np.array(self.block_contours, dtype=object)
        self.block_uvd = np.array(self.block_uvd, dtype = int)
        self.block_world_coords = np.array(self.block_world_coords, dtype = np.float32)
        self.block_dist = np.array(self.block_dist, dtype = np.float32)
        self.block_orients = np.array(self.block_orients, dtype = np.float32)
        self.block_sizes = np.array(self.block_sizes, dtype = int)  
        self.block_color_idxs = np.array(self.block_color_idxs, dtype = int)


    def reset(self):
        self.block_num = 0
        self.large_num = 0
        self.block_contours = []
        self.block_uvd = []
        self.block_world_coords = []
        self.block_dist = []
        self.block_orients = []
        self.block_sizes = []    # 0 for large, 1 for small
        self.block_color_idxs = []

    def sort_by_idx(self, indices, begin, end):
        self.block_contours[begin:end] = self.block_contours[begin:end][indices]
        self.block_uvd[begin:end] = self.block_uvd[begin:end][indices]
        self.block_world_coords[begin:end] = self.block_world_coords[begin:end][indices]
        self.block_dist[begin:end] = self.block_dist[begin:end][indices]
        self.block_orients[begin:end] = self.block_orients[begin:end][indices]
        self.block_sizes[begin:end] = self.block_sizes[begin:end][indices]
        self.block_color_idxs[begin:end] = self.block_color_idxs[begin:end][indices]

    def sort(self, key = "color"):
        if key == "dist":
            order = np.argsort(self.block_dist)
            self.sort_by_idx(order, 0, self.block_num)
        else:
            """ Sort block info by large to small and in rainbow order."""
            large_to_small = np.argsort(self.block_sizes)
            self.sort_by_idx(large_to_small, 0, self.block_num)
            #print("size sorted:")
            #print(self.block_color_idxs)
            large_rainbow_order = np.argsort(self.block_color_idxs[0:self.large_num])
            self.sort_by_idx(large_rainbow_order, 0, self.large_num)
            #print("large color sorted:")
            #print(self.block_color_idxs)
            small_rainbow_order = np.argsort(self.block_color_idxs[self.large_num:])
            #print("rainbor order")
            #print(small_rainbow_order)
            self.sort_by_idx(small_rainbow_order, self.large_num, self.block_num)
            #print("small color sorted:")
            #print(self.block_color_idxs)

class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720,1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.zeros((720,1280, 3)).astype(np.uint8)
        self.K_matrix = [[900.7150268554688, 0.0, 652.2869262695312], 
                            [0, 900.1925048828125,  358.359619140625], 
                            [0.0, 0.0, 1]]
        self.H_matrix = np.array([
                [0.99986783,  0.00297141,  0.01598385, -1],
                [0.,         -0.98315576,  0.18276969, 143],
                [0.0162577,  -0.18274553, -0.98302582, 1020],
                [ 0,            0,          0,          1  ]
            ])
        self.depth_K = K = np.array([
            [738.7109375, 0.0, 470.265625],
            [0.0, 738.8515625, 398.76171875],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.apriltags_centers = None
        self.camera_calibrated = False

        """ User defined parameters"""
        self.block_detection = BlockDetection()

        """ mouse clicks & calibration variables """
        self.homography_matrix = None
        self.intrinsic_matrix = np.eye(3)
        self.distortion_matrix = np.zeros(5)
        self.extrinsic_matrix = np.eye(4)
        self.last_click = np.array([0, 0]) # This contains the last clicked position
        self.new_click = False # This is automatically set to True whenever a click is received. Set it to False yourself after processing a click
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275], [-250, 275]]
        """ block info """
        self.DepthFrameWarped = np.zeros((720,1280)).astype(np.uint16)
        self.depth_high_thresh = 1001
        self.only_blocks = True


    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_detection.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormapped formats in HSV and RGB
        """
        if self.camera_calibrated and self.DepthFrameWarped.any():
            depth_camera_frame = self.DepthFrameWarped
        else:
            depth_camera_frame = self.DepthFrameRaw
        
        # Handling invalid depth values (if any)
        depth_camera_frame = np.nan_to_num(depth_camera_frame, nan=0, posinf=0, neginf=0)

        # Normalize depth for HSV hue channel
        depth_max = np.max(depth_camera_frame)
        depth_min = np.min(depth_camera_frame)
        if depth_max != depth_min:  # To avoid division by zero
            self.DepthFrameHSV[..., 0] = (depth_camera_frame).astype(np.uint8)  # Map to 0-179 range for Hue
        else:
            self.DepthFrameHSV[..., 0] = 0  # In case of constant depth, set Hue to 0
        
        # Set the saturation to maximum (full color intensity)
        self.DepthFrameHSV[..., 1] = 0xFF
        
        # Set the value (brightness) to a fixed value or some scaling based on depth range
        self.DepthFrameHSV[..., 2] = 0x9F  # This could be adjusted depending on your visualization needs
        
        # Convert HSV to RGB
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV, cv2.COLOR_HSV2RGB)

    def ColorizeDepthFrame2(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        if self.camera_calibrated and self.DepthFrameWarped.any():
            depth_camera_frame = self.DepthFrameWarped
        else:
            depth_camera_frame = self.DepthFrameRaw
        
        self.DepthFrameHSV[..., 0] = depth_camera_frame >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)   
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None


    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        if not self.camera_calibrated:
            self.K_matrix = [[900.7150268554688, 0.0, 652.2869262695312], 
                            [0, 900.1925048828125,  358.359619140625], 
                            [0.0, 0.0, 1]]

            self.H_matrix = np.array([
                [0.99986783,  0.00297141,  0.01598385, -1],
                [0.,         -0.98315576,  0.18276969, 143],
                [0.0162577,  -0.18274553, -0.98302582, 1020],
                [ 0,            0,          0,          1  ]
            ])

            self.depth_K = np.array([
                [738.7109375, 0.0, 470.265625],
                [0.0, 738.8515625, 398.76171875],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)

    def pixel2world_coord2(self, pixel_coord):

        pixel_frame = np.array([pixel_coord[0], pixel_coord[1]], dtype=np.float32)
        pixel_frame = pixel_frame.copy()
        K_init = np.array(self.intrinsic_matrix, dtype=np.float32)
        D_init = np.array(self.distortion_matrix, dtype=np.float32)

        undist_pixel = cv2.undistortPoints(pixel_frame, K_init, D_init)
        undist_pixel_h = np.array([undist_pixel[0][0][0],undist_pixel[0][0][1], 1.0], dtype=np.float32)
        #print(f'original{undist_pixel_h}')
        pixel_coordinates = np.matmul(undist_pixel_h, K_init.T) 

        
        #print(f'Original:{pixel_frame},New:{pixel_coordinates}')

        
        K_inv = np.linalg.inv(self.K_matrix)  # Inverse of the intrinsic matrix
        
        K_inv_scaled = K_inv*pixel_coord[2]
        
        camera_frame = np.dot(K_inv_scaled, np.array([pixel_coordinates[0], pixel_coordinates[1], 1], dtype=object))  
        
        # Step 2: Apply the inverse of the extrinsic transformation to get world coordinates
        # Inverse of rotation matrix R (rotation from world to camera)
        H_inv = np.linalg.inv(self.H_matrix)
        camera_frame = np.append(camera_frame, 1)

        # Apply the transformation
        world_coords = np.dot(H_inv, camera_frame)  # World coordinates
        return world_coords[:3]


    def pixel2world_coord(self, pixel_coord):
 
        pixel_frame = np.array([pixel_coord[0], pixel_coord[1], 1], dtype=object)

        
        K_inv = np.linalg.inv(self.K_matrix)  # Inverse of the intrinsic matrix
        K_inv_depth = np.linalg.inv(self.depth_K)  # Inverse of the intrinsic matrix
        
        K_inv_scaled = K_inv*(pixel_coord[2]-15)
        K_inv_depth_scaled = K_inv_depth*(pixel_coord[2]-15)
        
        camera_frame = np.dot(K_inv_scaled, pixel_frame) 
        camera_depth_frame =  np.dot(K_inv_depth_scaled, pixel_frame) 
        
        # Step 2: Apply the inverse of the extrinsic transformation to get world coordinates
        # Inverse of rotation matrix R (rotation from world to camera)
        H_inv = np.linalg.inv(self.H_matrix)

        camera_frame = np.append(camera_frame, 1)
        camera_depth_frame = np.append(camera_depth_frame, 1)

        # Apply the transformation
        world_coords = np.dot(H_inv, camera_frame)  # World coordinates
        world_coords_depth = np.dot(H_inv, camera_depth_frame)  # World coordinates

        final_coords = [world_coords[0],world_coords[1],world_coords_depth[2]]

        return final_coords

    """ Function used to detect if the block object is square or not. """
    def isBlock(self, contour):
        # Approximate the contour to a polygon with fewer vertices
        epsilon = 0.04 * cv2.arcLength(contour, True)  # Approximation accuracy
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated contour has 4 vertices (likely a rectangle or square)
        #if len(approx) == 4:
        # Check if the contour is a rectangle or square
        
        #x, y, w, h = cv2.boundingRect(approx)  # Get bounding box
        #aspect_ratio = float(w) / h  # Calculate the aspect ratio
        #print("Aspect ratio using boundingRect: %f" % aspect_ratio)
        
        rect = cv2.minAreaRect(contour)
        block_w_h = rect[1]

        s = np.max(block_w_h)
        area = cv2.contourArea(contour)
        s2 = area/s

        aspect_ratio = s/s2
        #print("Aspect ratio using minAreaRect: %f" % aspect_ratio)
        
        # Define thresholds for aspect ratio (1:1 is a square, ~1.5:1 for rectangular block)
        if (.9 <= aspect_ratio <= 1.55) or not self.only_blocks:  # Rough threshold for square
            return True
        else:
            return False
    
    def blockDetector(self, warped_rgb, warped_depth):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        if not self.camera_calibrated:
            pass
        ret_img = self.detectBlocksInDepthImage(warped_rgb, warped_depth)

        return ret_img

    def retrieve_area_color(self, data, contour, colors, colors_hsv):
        mask = np.zeros(data.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mean = cv2.mean(data, mask=mask)[:3]
        mean_bgr = np.array([[mean]], dtype=np.uint8)
        mean_hsv = cv2.cvtColor(mean_bgr, cv2.COLOR_BGR2HSV)

        min_dist = (np.inf, None)
        #print(mean_hsv)
        for c_hsv in colors_hsv:
            if c_hsv["id"] == 'red':
                if self.is_color_in_range(mean_hsv, c_hsv["lower_bound2"], c_hsv["upper_bound2"]):
                    #print("is red")
                    return c_hsv["id"]
            if self.is_color_in_range(mean_hsv, c_hsv["lower_bound"], c_hsv["upper_bound"]):
                #print("is", c_hsv["id"])
                return c_hsv["id"]
        return 'None'
    
    def is_color_in_range(self, hsv_color, lower_bound, upper_bound):
        """
        Checks if an HSV color falls within a specified range.

        Args:
            hsv_color: A tuple or list representing the HSV color (H, S, V).
            lower_bound: A tuple or list representing the lower bound of the HSV range (H, S, V).
            upper_bound: A tuple or list representing the upper bound of the HSV range (H, S, V).

        Returns:
            True if the color is within the range, False otherwise.
        """
        hsv_color_array = np.array(hsv_color, dtype=np.uint8)
        lower_bound_array = np.array(lower_bound, dtype=np.uint8)
        upper_bound_array = np.array(upper_bound, dtype=np.uint8)

        mask = cv2.inRange(hsv_color_array, lower_bound_array, upper_bound_array)
        return np.any(mask)

    def detectBlocksInDepthImage(self, warped_rgb, warped_depth):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        self.block_detection.reset()
        
        colors = list((
            {'id': 'red', 'color': (10, 10, 127)},
            {'id': 'orange', 'color': (30, 75, 150)},
            {'id': 'yellow', 'color': (30, 150, 200)},
            {'id': 'green', 'color': (20, 60, 20)},
            {'id': 'blue', 'color': (100, 50, 0)},
        #    {'id': 'violet', 'color': (100, 40, 80)})
        ))
        red = np.array([[[10, 10, 127]]], dtype=np.uint8)
        orange = np.array([[[30, 75, 150]]], dtype = np.uint8)
        yellow = np.array([[[30, 150, 200]]], dtype = np.uint8)
        green = np.array([[[20, 60, 20]]], dtype = np.uint8)
        blue = np.array([[[100, 50, 0]]], dtype = np.uint8)
        #violet = np.array([[[100, 40, 80]]], dtype = np.uint8)

        colors_hsv = list((
            {'id': 'red', 'lower_bound': (0, 70, 60), 'upper_bound': (3, 255, 255), 'lower_bound2': (170, 70, 60), 'upper_bound2': (180, 255, 255)},
            {'id': 'orange', 'lower_bound': (3, 100, 60), 'upper_bound': (20, 255, 255)},
            {'id': 'yellow', 'lower_bound': (20, 100, 60), 'upper_bound': (35, 255, 255)},
            {'id': 'green', 'lower_bound': (35, 65, 60), 'upper_bound': (85, 255, 255)},
            {'id': 'blue', 'lower_bound': (100, 70, 60), 'upper_bound': (110, 255, 255)},
            {'id': 'violet', 'lower_bound': (110, 10, 50), 'upper_bound': (170, 255, 255)})
        )
        color_id = ['red', 'orange', 'yellow', 'green', 'blue', 'violet', 'None']
        #color_id = ['red', 'orange', 'yellow', 'green', 'blue']

        lower = int(953)    # Threashold of the lower bound (depth)
        upper = int(self.depth_high_thresh)    # Threashold of the upper bound (depth)


        bgr_image = cv2.cvtColor(warped_rgb, cv2.COLOR_RGB2BGR)
        cnt_image = warped_rgb #cv2.cvtColor(warped_rgb, cv2.COLOR_RGB2BGR)
        depth_data = warped_depth
        
        """mask out arm & outside board"""
        mask = np.zeros_like(depth_data, dtype=np.uint8)
        cv2.rectangle(mask, (130,90),(1140,670), 255, cv2.FILLED)
        cv2.rectangle(mask, (550,430),(723,670), 0, cv2.FILLED) #355
        cv2.rectangle(cnt_image, (130,90),(1140,670), (255, 0, 0), 2)
        cv2.rectangle(cnt_image, (550,430),(723,670), (255, 0, 0), 2)
        if not self.only_blocks:
            cv2.rectangle(mask, (950,240),(1140,430), 255, cv2.FILLED)
            cv2.rectangle(cnt_image, (950,240),(1140,430), (255, 0, 0), 2)

        thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.imwrite('mask.jpg', mask)
        #cv2.imwrite('threashold.jpg', thresh)
        for contour in contours:
            # Reject shapes that are not cubes
            #if not self.isBlock(contour):
            #    continue
            M = cv2.moments(contour)
            # Reject false positive detections by area size
            if M['m00'] < 200 or abs(M["m00"]) > 7000:
                continue
            if M['m00']:
                # Reject false positive detections by aspect ratio
                #cv2.drawContours(cnt_image, contour, -1, (255,255,255), thickness=1)
                if not self.isBlock(contour):
                    continue
                
                # Get color of the block
                color = self.retrieve_area_color(bgr_image, contour, colors, colors_hsv)
                if color == 'None' and self.only_blocks:
                    continue
                color_idx = color_id.index(color)

                # Get bounding box of the block
                rect = cv2.minAreaRect(contour)
                block_center = rect[0]
                block_w_h = rect[1]
                block_ori = rect[2]
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # Get center of the block
                #cx_b = int(block_center[0])
                #cy_b = int(block_center[1])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cz = depth_data[cy, cx]

                # Convert pixel coordinates to world coordinates using the homography matrix
                pt_img_frame = np.linalg.inv(self.homography_matrix) @ np.array([[cx], [cy], [1]])
                lamda = pt_img_frame[2]
                block_world_coord = self.pixel2world_coord([pt_img_frame[0]/lamda, pt_img_frame[1]/lamda, self.DepthFrameRaw[int(pt_img_frame[1]/lamda)][int(pt_img_frame[0]/lamda)]])
                

                # Save detected block info
                area = cv2.contourArea(contour)
                #print(area)
                if area < 1500:
                    self.block_detection.block_sizes.append(1)  # small block
                else:
                    self.block_detection.block_sizes.append(0)  # large block
                    self.block_detection.large_num += 1
                    
                dist = np.sqrt(block_world_coord[0]**2 + block_world_coord[1]**2)

                self.block_detection.block_uvd.append([cx, cy, cz])
                self.block_detection.block_contours.append(contour)
                self.block_detection.block_num += 1                
                self.block_detection.block_world_coords.append(block_world_coord)
                self.block_detection.block_dist.append(dist[0])
                self.block_detection.block_orients.append(np.deg2rad(block_ori))
                self.block_detection.block_color_idxs.append(color_idx)
                angle = np.rad2deg(np.arctan2(block_world_coord[0],block_world_coord[1]))
                
                # Draw block and label color on the image
                cv2.circle(cnt_image, (cx, cy), 2, (255, 0, 0), 2)  # red center from cv2.moments(contour)
                #cv2.circle(cnt_image, (cx_b, cy_b), 2, (0, 255, 0), 2)  # green center from cv2.minAreaRect(contour)
                cv2.drawContours(cnt_image,[box],0,(0,0,255), 1)
                cv2.drawContours(cnt_image, contour, -1, (0,255,255), thickness=1)
                cv2.putText(cnt_image, f'{color}', (cx-30, cy+40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), thickness=2)
                #cv2.putText(cnt_image, f'{color},{rect[2]},{angle}', (cx-30, cy+40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), thickness=2)
                #cv2.putText(cnt_image, str(int(block_ori)), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), thickness=2)
        
        #cv2.imwrite('detected.jpg', cnt_image)
        if not self.block_detection.block_num == 0:
            self.block_detection.update()
            self.block_detection.sort(self.block_detection.sort_key)

        """
        print("Color index blocks:")
        print(self.block_detection.block_color_idxs)
        print("Orient blocks:")
        print(self.block_detection.block_orients)
        print("Total number:")
        print(self.block_num)
        print("Large number:")
        print(self.large_num)
        """
        return cnt_image



    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        modified_image = self.VideoFrame.copy()
        # Write your code here
        if self.H_matrix is None or self.homography_matrix is None:
            return
        
        x = self.grid_points[0].flatten()
        y = self.grid_points[1].flatten()

        world_coords = [x, y, np.zeros_like(x), np.ones_like(x)]
        camera_coords = self.H_matrix @ world_coords
        image_coords = self.K_matrix @ camera_coords[:-1, :]

        image_coords = self.homography_matrix @ image_coords
 
        for i in range(np.shape(image_coords)[1]):
            z = image_coords[2][i]
            x = image_coords[0][i] / z
            y = image_coords[1][i] / z


            modified_image = cv2.circle(modified_image, (int(x), int(y)), 5, (0, 0, 225), 2)

        self.GridFrame = modified_image
     
    def drawTagsInRGBImage(self, msg):
        """
        @brief      Draw tags from the tag detection

                    TODO: Use the tag detections output, to draw the corners/center/tagID of
                    the apriltags on the copy of the RGB image. And output the video to self.TagImageFrame.
                    Message type can be found here: /opt/ros/humble/share/apriltag_msgs/msg

                    center of the tag: (detection.centre.x, detection.centre.y) they are floats
                    id of the tag: detection.id
        """
        modified_image = self.VideoFrame.copy()

        self.apriltags_centers = []
        
        AprilTagDetectionArray = msg.detections
        for i in range(len(AprilTagDetectionArray)):
            detection = AprilTagDetectionArray[i]
            
            # Draw center
            center = (int(detection.centre.x), int(detection.centre.y))
            self.apriltags_centers.append(center)
            modified_image = cv2.circle(modified_image, center, 3, (0, 0, 255), -1)

            # Draw ID
            modified_image = cv2.putText(modified_image, "ID: %i" %detection.id, (center[0]+25, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2, cv2.LINE_AA)
            
            # Draw edges
            corners = detection.corners
            for j in range(len(corners)):
                start_p = corners[j]
                start_coord = (int(start_p.x), int(start_p.y))
                end_p = corners[(j+1)%len(corners)]
                end_coord = (int(end_p.x), int(end_p.y))
                modified_image = cv2.line(modified_image, start_coord, end_coord, (0, 0, 255), 2)


        self.apriltags_centers = np.array(self.apriltags_centers)

        self.TagImageFrame = modified_image

    def recover_homogenous_transform_pnp(self,image_points, world_points, K, D):
        '''
        Use SolvePnP to find the rigidbody transform representing the camera pose in
        world coordinates (not working)
        '''

        image_points_f = np.array(image_points, dtype=float)
        world_points_f = np.array(world_points, dtype=float)

        # print(K)

        # image_points_f_depth = []

        # for ps in image_points_f:
        #     z=self.DepthFrameRaw[int(ps[1])][int(ps[0])]
        #     print(z)
        #     temp = [ps[0], ps[1], z]
        #     image_points_f_depth.append(temp)

        # image_points_f_depth = np.stack(image_points_f_depth, axis=0)

        # print(image_points_f_depth)
        # print(world_points_f)

        distCoeffs = D
        [_, R_exp, t] = cv2.solvePnP(world_points_f,
                                    image_points_f,
                                    K,
                                    distCoeffs)
        R, _ = cv2.Rodrigues(R_exp)
        return np.row_stack((np.column_stack((R, t)), (0, 0, 0, 1)))
    
    def recover_homogeneous_transform_svd(self, m, d):
        ''' 
        finds the rigid body transform that maps m to d: 
        d == np.dot(m,R) + T
        http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
        '''
        image_points_f = np.array(m, dtype=float)
        world_points_f = np.array(d, dtype=float)

        image_points_f_depth = []

        for ps in image_points_f:
            z=self.DepthFrameRaw[int(ps[1])][int(ps[0])]
            print(z)
            temp = [ps[0], ps[1], z]
            image_points_f_depth.append(temp)

        image_points_f_depth = np.stack(image_points_f_depth, axis=0)

        print(image_points_f_depth)
        print(world_points_f)


        # calculate the centroid for each set of points
        d_bar = np.sum(d, axis=0) / np.shape(d)[0]
        m_bar = np.sum(image_points_f_depth, axis=0) / np.shape(m)[0]

        # we are using row vectors, so tanspose the first one
        # H should be 3x3, if it is not, we've done this wrong
        H = np.dot(np.transpose(d - d_bar), image_points_f_depth - m_bar)
        [U, S, V] = np.linalg.svd(H)

        R = np.matmul(V, np.transpose(U))
        # if det(R) is -1, we've made a reflection, not a rotation
        # fix it by negating the 3rd column of V
        if np.linalg.det(R) < 0:
            V = [1, 1, -1] * V
            R = np.matmul(V, np.transpose(U))
        T = d_bar - np.dot(m_bar, R)
        return np.transpose(np.column_stack((np.row_stack((R, T)), (0, 0, 0, 1))))


    def recover_homogeneous_transform_svd_modified(self, m, d):
        ''' 
        finds the rigid body transform that maps m to d: 
        d == np.dot(m,R) + T
        http://graphics.stanford.edu/~smr/ICP/comparison/eggert_comparison_mva97.pdf
        This modification recalculates R after finding T, which is technically incorrect but in practice works
        '''

        image_points_f = np.array(m, dtype=float)
        world_points_f = np.array(d, dtype=float)

        image_points_f_depth = []

        for ps in image_points_f:
            z=self.DepthFrameRaw[int(ps[1])][int(ps[0])]
            print(z)
            temp = [ps[0], ps[1], z]
            image_points_f_depth.append(temp)

        image_points_f_depth = np.stack(image_points_f_depth, axis=0)

        print(image_points_f_depth)
        print(world_points_f)

        # calculate the centroid for each set of points
        d_bar = np.sum(d, axis=0) / np.shape(d)[0]
        m_bar = np.sum(image_points_f_depth, axis=0) / np.shape(m)[0]

        # we are using row vectors, so tanspose the first one
        # H should be 3x3, if it is not, we've done this wrong
        H = np.dot(np.transpose(d - d_bar), image_points_f_depth - m_bar)
        [U, S, V] = np.linalg.svd(H)

        R = np.matmul(V, np.transpose(U))

        # Get translation
        T = d_bar - np.dot(m_bar, R)

        # if det(R) is -1, we've made a reflection, not a rotation
        # fix it by negating the 3rd column of V
        if np.linalg.det(R) < 0:
            V = [1, 1, -1] * V
            R = np.matmul(V, np.transpose(U))
        return np.transpose(np.column_stack((np.row_stack((R, T)), (0, 0, 0, 1))))


    def homography_transform(self):
        if not self.camera_calibrated:
            # Select source points to apply the homography transform from
            src_pts = np.array(self.apriltags_centers[0:4]).reshape((4,2))

            # Select destination points to apply the homography transform to
            dest_pts = np.array([[362, 528], 
                               [918, 528],
                               [918, 240],
                               [362, 240]])
           
            # dest_pts = np.array([100, 650, 
            #     650, 650,
            #     650, 100,
            #     100, 100,]).reshape((4, 2))


            self.homography_matrix = cv2.findHomography(src_pts, dest_pts)[0]
            
        #warp_img = cv2.warpPerspective(new_img, self.homography_matrix, (new_img.shape[1], new_img.shape[0]))
        
        """
        # Draw green dots to represent the source points
        for pt in src_pts:
            cv2.circle(image, tuple(pt), 5, (0, 255, 0), -1)

        # Draw red dots to represent the destination points
        for pt in dest_pts:
            cv2.circle(new_img, tuple(pt), 5, (0, 0, 255), -1)
        """
        #cv2.imwrite("homography_image.png", warp_img)
        #return warp_img

    def transform_images2(self, color_image, depth_image):
        transformed_color_image = np.zeros_like(color_image)
        transformed_depth_image = np.zeros_like(depth_image)


        K = self.K_matrix
        
        T_h = self.H_matrix


        # Iterate over each pixel in the depth image
        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                # Get the depth value for the pixel
                Z = depth_image[v, u]  # Depth value (Z)
                if Z == 0:  # Skip invalid depth values (no information)
                    continue

                # Compute the 3D coordinates in the camera frame
                X = (u - K[0, 2]) * Z / K[0, 0]
                Y = (v - K[1, 2]) * Z / K[1, 1]

                # Create a 3D point (in homogeneous coordinates)
                world_point = np.array([X, Y, Z, 1.0])

                # Apply the homogeneous transformation matrix to the 3D point
                transformed_point = np.dot(T_h, world_point)

                # Project the transformed 3D point back to 2D image coordinates
                u_prime = K[0, 0] * transformed_point[0] / transformed_point[2] + K[0, 2]
                v_prime = K[1, 1] * transformed_point[1] / transformed_point[2] + K[1, 2]

                # Check if the transformed pixel is within the image bounds
                if 0 <= u_prime < color_image.shape[1] and 0 <= v_prime < color_image.shape[0]:
                    u_prime, v_prime = int(u_prime), int(v_prime)

                    # Map the transformed depth value (or color) to the new position
                    transformed_depth_image[v_prime, u_prime] = Z
                    transformed_color_image[v_prime, u_prime] = color_image[v, u]

        return transformed_color_image, transformed_depth_image

    def transform_images(self, rgb_img, depth_img):
        """
        Transforms the RGB and depth images to a new viewpoint based on the provided camera extrinsic matrices.

        Parameters:
            rgb_img (np.array): Original RGB image.
            depth_img (np.array): Original depth image.
            K (np.array): Camera intrinsic matrix (3x3).
            T_i (np.array): Initial camera extrinsic matrix (4x4) representing the pose of the camera in the world frame.
            T_f (np.array): Final camera extrinsic matrix (4x4) representing the desired pose of the camera in the world frame.

        Returns:
            tuple: Transformed RGB image, Transformed depth image.
        """

        K = self.depth_K
        #print(K)
        
        T_i = self.H_matrix

        T_f = np.array([
                [1, 0,  0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 1000],   
                [0, 0,  0, 1]
                ]) # Final camera extrinsic matrix (desired pose of the camera in the world frame)

        # Use to ensure that the entire transformed image is contained within the output
        scale_factor=1.0

        h, w = rgb_img.shape[:2]
        
        # Calculate the relative transformation matrix between the initial and final camera poses
        T_relative = np.dot(T_f, np.linalg.inv(T_i))


        # Compute the homography for RGB image
        H_rgb = self.homography_matrix

        # Create a larger canvas for RGB
        enlarged_h_rgb, enlarged_w_rgb = int(h * scale_factor), int(w * scale_factor)
        
        # Warp the RGB image using the computed homography onto the larger canvas
        warped_rgb = cv2.warpPerspective(rgb_img, H_rgb, (enlarged_w_rgb, enlarged_h_rgb))
        
        # For the depth values, we first transform them to 3D points, apply the T_relative transformation, and then project them back to depth values
        # Back-project to 3D camera coordinates
        u = np.repeat(np.arange(w)[None, :], h, axis=0)
        v = np.repeat(np.arange(h)[:, None], w, axis=1)
        
        Z = np.array(depth_img)

        X = (u - K[0,2]) * Z / K[0,0]
        Y = (v - K[1,2]) * Z / K[1,1]
        
        # Homogeneous coordinates in the camera frame
        points_camera_frame = np.stack((X, Y, Z, np.ones_like(Z)), axis=-1)
        
        # Apply the relative transformation to the depth points
        points_transformed = np.dot(points_camera_frame, T_relative.T)
        
        # Project back to depth values
        depth_transformed = points_transformed[..., 2]
        
        # Create a larger canvas for depth
        enlarged_h_depth, enlarged_w_depth = int(h * scale_factor), int(w * scale_factor)

        #cv2.namedWindow("window window", cv2.WINDOW_NORMAL)
        #cv2.imshow('window window', rgb_img)
        
        # Use the same homography as RGB for depth
        warped_depth = cv2.warpPerspective(depth_transformed, H_rgb, (enlarged_w_depth, enlarged_h_depth))
        
        return warped_rgb, warped_depth

class ImageListener(Node):
    def __init__(self, topic, camera):
        super().__init__('image_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        #print(self.camera.calibrated)
        if self.camera.camera_calibrated:
            #img = self.camera.homography_transform(cv_image)
            warped_rgb, warped_depth = self.camera.transform_images(cv_image, self.camera.DepthFrameRaw)
            self.camera.DepthFrameWarped = warped_depth
            time.sleep(0.1)
            img = self.camera.blockDetector(warped_rgb, warped_depth)  
        else:
            img = cv_image
        
        self.camera.VideoFrame = img
            #self.camera.VideoFrame = cv_image


class TagDetectionListener(Node):
    def __init__(self, topic, camera):
        super().__init__('tag_detection_listener')
        self.topic = topic
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        self.camera.tag_detections = msg
        if np.any(self.camera.VideoFrame != 0):
            self.camera.drawTagsInRGBImage(msg)


class CameraInfoListener(Node):
    def __init__(self, topic, camera):
        super().__init__('camera_info_listener')  
        self.topic = topic
        self.tag_sub = self.create_subscription(CameraInfo, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.k, (3, 3))
        self.camera.distortion_matrix = np.reshape(data.d, (1, 5))
        #print(self.camera.intrinsic_matrix)


class DepthListener(Node):
    def __init__(self, topic, camera):
        super().__init__('depth_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)

        self.camera.DepthFrameRaw = cv_depth

        # self.camera.DepthFrameRaw = self.camera.DepthFrameRaw / 2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_detection_topic = "/detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(image_listener)
        self.executor.add_node(depth_listener)
        self.executor.add_node(camera_info_listener)
        self.executor.add_node(tag_detection_listener)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        try:
            while rclpy.ok():
                #self.camera.blockDetector()
                start_time = time.time()
                rgb_frame = self.camera.convertQtVideoFrame()
                depth_frame = self.camera.convertQtDepthFrame()
                tag_frame = self.camera.convertQtTagImageFrame()
                self.camera.projectGridInRGBImage()
                grid_frame = self.camera.convertQtGridFrame()
                if ((rgb_frame != None) & (depth_frame != None)):
                    self.updateFrame.emit(
                        rgb_frame, depth_frame, tag_frame, grid_frame)
                self.executor.spin_once() # comment this out when run this file alone.
                elapsed_time = time.time() - start_time
                sleep_time = max(0.03 - elapsed_time, 0)
                time.sleep(sleep_time)

                if __name__ == '__main__':
                    cv2.imshow(
                            "Image window",
                            cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                    cv2.imshow(
                        "Tag window",
                        cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Grid window",
                        cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(3)
                    time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        
        self.executor.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    try:
        camera = Camera()
        videoThread = VideoThread(camera)
        videoThread.start()
        try:
            videoThread.executor.spin()
        finally:
            videoThread.executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()