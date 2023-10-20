from matplotlib import pyplot as plt
from enum import Enum
import numpy as np
import open3d as o3d
import cv2
import pyrealsense2
import csv
import heapq
import matplotlib
import copy
"""
The next part is for defining constants
these constants are used to make sure the water case doesnt get damaged
only change these if you know what you are doing
"""
#150 140 0.3
#Constants
TOP_PLANE_SAFE_DISTANCE = 0.005
LS_WATER_CASE = -0.27 #Left side water case //waterbak lange kant
RS_WATER_CASE = 0.27 #Right side water case //waterbak lange kant
TS_WATER_CASE = 0.12 #Top side water case //waterbak korte kant
BS_WATER_CASE = -0.12 #Bottom side water case //waterbak korte kant
SCAN_HEAD = 0.028 #Scan head size
PRE_DEFINED_DISTANCE = -0.34 #Pre defined distance from the camera to the object

#Accessing the point cloud
pcd = o3d.io.read_point_cloud("test.ply")

#Function for exceueting the path generation
def path_generation():

    #array's
    x_values = []
    y_values = []
    z_values = []
    wayPoints = []
    safety_points = []
    #Variables
    T_move = 0 # top plane move
    T_calculate = 0 # top plane calculate
    #logic
    # T_move = round(calculate_top_plane(300) + TOP_PLANE_SAFE_DISTANCE, 4)
    # print(T_move, "T_move")
    T_move = PRE_DEFINED_DISTANCE + TOP_PLANE_SAFE_DISTANCE
    print(T_move, "T_move3")
    T_calculate = T_move - (TOP_PLANE_SAFE_DISTANCE * 2)
    print(T_calculate, "test")
    Create_waypoints(x_values, y_values, T_calculate, T_move, wayPoints, safety_points)
    Write_to_csv(x_values, y_values, wayPoints, safety_points)
    print("Done")

#Function for Calculating the top plane
def calculate_top_plane(sample_amount):

    z_cords = []
    z_average = 0
    for cords_all in pcd.points:
        if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
            z_cords.append(cords_all[2])
    z_average = heapq.nlargest(sample_amount, z_cords)
    z_average = np.mean(z_average)

    return z_average

def Draw_Cords(x_values, y_values):
                    
    plt.plot(x_values, y_values, 'ro')
    plt.axis([-0.2, 0.3, -0.2, 0.3])

    for i_x, i_y in zip(x_values, y_values):
        plt.text(i_x, i_y, '({}, {})'.format(i_x, i_y))

    plt.show()

#Function for creating the waypoints
def Create_waypoints(x_values, y_values, T_calculate, T_move, wayPoints, safety_points):

    Create_outer_waypoints(T_calculate, x_values, y_values)
    fill_waypoints(x_values, y_values, wayPoints, T_move)
    Add_safety_points(wayPoints, safety_points)

#Function for creating waypoints on the left and right side of the top plane
def Create_outer_waypoints(top_plane_calculate, x_values, y_values):

    #Local variables
    new_left_side = 100
    new_right_side = -100
    new_y_cord = Get_lowest_point(top_plane_calculate)
    even = 0
    
    while new_y_cord + (SCAN_HEAD / 4) <= Get_highest_point(top_plane_calculate):
        even += 1
        """
        This part adds the y cords to the array 2 times,
        this is done to because the x cord receives 2 values (left and right side)
        These are both on the same y cord
        """
        A = 0
        while A <= 1:
            rounded_new_Y = (round(new_y_cord, 4) - 0.01)
            y_values.append(rounded_new_Y)
            A += 1
        """
        Finding the outer points
        """
        for cords_all in pcd.points:
            if cords_all[2] > top_plane_calculate:
                if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
                    if cords_all[1] >= new_y_cord and cords_all[1] < new_y_cord + 0.002:
                        # get the most left points
                        if cords_all[0] < new_left_side:
                            new_left_side = cords_all[0]
                        # get the most right points
                        if cords_all[0] > new_right_side:
                            new_right_side = cords_all[0]

        rounded_left_point = (round(new_left_side, 4) - 0.02)
        rounded_right_point = (round(new_right_side, 4) + 0.02)
        """
        switch the way arrays get filled for zigzag pattern
        """
        if even % 2 == 1:
            x_values.append(rounded_right_point) 
            Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even)
            x_values.append(rounded_left_point)
        elif even % 2 == 0:
            x_values.append(rounded_left_point)
            Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even)
            x_values.append(rounded_right_point) 
        new_y_cord += SCAN_HEAD

    return x_values, y_values

#Function for creating waypoint's between the outer points
def Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even):

    if even % 2 == 1:
        new_x_point = rounded_right_point
        while new_x_point > rounded_left_point:
            new_x_point = round(new_x_point - 0.01, 4)
            x_values.append(new_x_point) 
            y_values.append(rounded_new_Y)
    elif even % 2 == 0:
        new_x_point = rounded_left_point
        while new_x_point < rounded_right_point:
            new_x_point = round(new_x_point + 0.01, 4)
            x_values.append(new_x_point) 
            y_values.append(rounded_new_Y)

#Function for ordering the waypoints in the right order
def fill_waypoints(x_values, y_values, wayPoints, top_plane_move):
    """
    ordering algorithm
    """          
    for h in range(len(x_values)):
        wayPoints.append([round(x_values[h], 4), round(y_values[h], 4), top_plane_move, 0, 0, 0])

#Function for getting the leftest point of the top plane
def Get_leftest_point(top_plane_calculate):
    most_left_point = 100
    for cords_all in pcd.points:
            if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
                if cords_all[2] > top_plane_calculate:
                    if cords_all[0] < most_left_point:
                        most_left_point = cords_all[0]
    return most_left_point

#Function for getting the rightest point of the top plane
def Get_rightest_point(top_plane_calculate):
    most_right_point = -100
    for cords_all in pcd.points:
        if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
            if cords_all[2] > top_plane_calculate:
                if cords_all[0] > most_right_point:
                    most_right_point = cords_all[0]
    return most_right_point

#Function for getting the highest point of the top plane
def Get_highest_point(top_plane_calculate):
    highest_point_y = -100
    for cords_all in pcd.points:
        if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
            if cords_all[2] > top_plane_calculate:
                if cords_all[1] > highest_point_y:
                    highest_point_y = cords_all[1]
    return highest_point_y

#Function for getting the lowest point of the top plane
def Get_lowest_point(top_plane_calculate):
    lowest_point_y = 100
    for cords_all in pcd.points:
        if LS_WATER_CASE < cords_all[0] < RS_WATER_CASE and BS_WATER_CASE < cords_all[1] < TS_WATER_CASE:
            if cords_all[2] > top_plane_calculate:
                if cords_all[1] < lowest_point_y:
                    lowest_point_y = cords_all[1]
    return lowest_point_y


#Function for creating points making sure the cobot arm doesnt hit the water box
def Add_safety_points(wayPoints, safety_points):

    copy_list = copy.deepcopy(wayPoints)
    copy_list2 = copy.deepcopy(wayPoints)
    first_row = copy_list[0]
    last_row = copy_list[-1]
    lastlast_row = copy_list2[-1]
    safety_points.append(first_row)
    print(first_row)
    safety_points.append(last_row)
    safety_points.append(lastlast_row)
    safety_points[0][2] = 0
    safety_points[1][2] = -0.25
    safety_points[2][2] = 0

#Function making sure the cords being wirtten to the csv are always inside the water box
def Final_Safety_check_Cords(wayPoints):

    for i in range(len(wayPoints)):
        print(i)
        if LS_WATER_CASE < wayPoints[i][0] < RS_WATER_CASE and BS_WATER_CASE < wayPoints[i][1] < TS_WATER_CASE:
            print("Cords are in the water")
        else:
            print("Cords are not in the water")
            return False
    return wayPoints

# function for writing the cordinates to a csv file
def Write_to_csv(x_values, y_values, wayPoints, safety_points):
    
    firstfirst_row = [0, 0 ,-0.2, 0, 0, 0]
    null_row = [0,0,0,0,0,0]
    first_row = safety_points[0]
    last_row = safety_points[1]
    lastlast_row = safety_points[2]

    print(len(wayPoints))

    header = ['x', 'y', 'z', "rx", "ry", "rz"]

    Draw_Cords(x_values, y_values)

    with open('cords.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=';')
        writer.writerow(header)
        writer.writerow(firstfirst_row)
        writer.writerow(first_row)
        writer.writerows(Final_Safety_check_Cords(wayPoints)) 
        writer.writerow(last_row)
        writer.writerow(lastlast_row)
        writer.writerow(null_row)

def main():
    print("ik wordt uitgevoerd")
    path_generation()

if __name__ == '__main__':
    main()