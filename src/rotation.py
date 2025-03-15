import math

def calculate_orientation(x, y, z):
    # Calculate pitch and roll using the accelerometer readings
    pitch = math.atan2(y, math.sqrt(x**2 + z**2))
    roll = math.atan2(x, math.sqrt(y**2 + z**2))
    
    # Convert radians to degrees
    pitch_degrees = math.degrees(pitch)
    roll_degrees = math.degrees(roll)
    
    return pitch_degrees, roll_degrees

import numpy as np

def rotation_matrix(pitch, roll):
    # Convert angles from degrees to radians
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    
    # Compute the rotation matrix
    R = np.array([
        [math.cos(roll), math.sin(pitch) * math.sin(roll), math.cos(pitch) * math.sin(roll)],
        [0, math.cos(pitch), -math.sin(pitch)],
        [-math.sin(roll), math.cos(roll) * math.sin(pitch), math.cos(pitch) * math.cos(roll)]
    ])
    
    return R


# Example accelerometer readings
x = -0.157
y = 1.765
z = -9.493

# Get the orientation (pitch, roll)
pitch, roll = calculate_orientation(x, y, z)
print(f"Pitch: {pitch:.2f} degrees")
print(f"Roll: {roll:.2f} degrees")

# Get the rotation matrix
R = rotation_matrix(pitch, roll)
print("Rotation Matrix:")
print(R)


def rotate_y(matrix, theta):
    # Create the rotation matrix for rotating about the Y axis
    rotation_matrix = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    
    # Multiply the rotation matrix with the input matrix
    rotated_matrix = np.dot(matrix, rotation_matrix.T)  # Transpose to match dimensions
    return rotated_matrix


r_2 = rotate_y(R, np.deg2rad(180))

print(r_2)

def rotate_x(matrix, theta):
    # Create the rotation matrix for rotating about the X axis
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])
    
    # Multiply the rotation matrix with the input matrix
    rotated_matrix = np.dot(matrix, rotation_matrix.T)  # Transpose to match dimensions
    return rotated_matrix



r_3 = rotate_x(R, np.deg2rad(180))

print(r_3)
