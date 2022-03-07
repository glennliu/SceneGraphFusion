import numpy as np


root_path = '/home/lch/dataset/sceneNN/raw_data/096'
trajectory_path = root_path + '/trajectory.log'
file1 = open(trajectory_path, 'r')
Lines = file1.readlines()

# Read
with open(trajectory_path) as f:
    raw_trajectory = [[float(num) for num in line.split(' ')] for line in f]
num_lines = len(raw_trajectory)

row=0
transformation_trajectory = []
while(row<num_lines):
    # T = np.zeros((4,4))
    line = raw_trajectory[row]
    if(len(line)==3):
        T = raw_trajectory[row+1:row+5]
        transformation_trajectory.append(T)
    row =row+5


# Write
num_pose = len(transformation_trajectory)
count = 0
for T in transformation_trajectory:
    output_file_dir = root_path + '/pose/camera_pose_' + str(count).zfill(5) + '.txt'
    np.savetxt(output_file_dir, T,fmt='%.6f',delimiter=' ')
    count += 1
