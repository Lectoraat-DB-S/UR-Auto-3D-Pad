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
#Constants
TOP_PLANE_SAFE_DISTANCE = 0.005
LS_WATER_CASE = -0.75 #Left side water case
RS_WATER_CASE = 0.75 #Right side water case
TS_WATER_CASE = 0.600 #Top side water case
BS_WATER_CASE = -0.600 #Bottom side water case
SCAN_HEAD = 0.028 #Scan head size
PRE_DEFINED_DISTANCE = -0.34 #Pre defined distance from the camera to the object
SEARCH_AREA = 0.005 #the amount of distance from the x,y cords to search for a Z value

#Accessing the point cloud
pcd = o3d.io.read_point_cloud("test.ply")

#Function for exceueting the path generation
def path_generation():

    #array's
    x_values = []
    y_values = []
    z_values = []
    rx_values = []
    ry_values = []
    rz_values = []
    wayPoints = []
    safety_points = []
    #Variables
    T_move = 0 # top plane move
    T_calculate = 0 # top plane calculate
    #logic
    # T_move = round(calculate_top_plane(300) + TOP_PLANE_SAFE_DISTANCE, 4)
    T_move = PRE_DEFINED_DISTANCE + TOP_PLANE_SAFE_DISTANCE
    T_calculate = T_move - (TOP_PLANE_SAFE_DISTANCE * 2)
    Create_waypoints(x_values, y_values, T_calculate, T_move, wayPoints, z_values, rx_values, ry_values, rz_values)
    Add_safety_points(wayPoints, safety_points)
    Write_to_csv(wayPoints, safety_points, x_values, y_values)
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
def Create_waypoints(x_values, y_values, T_calculate, T_move, wayPoints, z_values, rx, ry, rz):

    Create_outer_waypoints(T_calculate, x_values, y_values, z_values, T_move)
    Calculate_rotation(x_values, z_values, rx, ry, rz)
    print(rx, "rx")
    fill_waypoints(x_values, y_values, wayPoints, z_values, rx, ry, rz)

#Function for calculating the roation of the coordinates
def Calculate_rotation(x_values, z_value, rx, ry, rz):
    #use the x and y cords to calculate the rotation to the next waypoint
    for i in range(len(x_values) - 1):
        #calculate the rotation
        c = x_values[i + 1] - x_values[i]
        a = z_value[i]

        cornerD = np.arctan(a / c)
        print(cornerD)
        corenerE = 90 - cornerD
        print(corenerE)

        #add the rotation to the array
        rx.append(corenerE)
        ry.append(0)
        rz.append(0)
        if i == len(x_values) - 2:
            rx.append(0)
            ry.append(0)
            rz.append(0)  

#Function for creating waypoints on the left and right side of the top plane
def Create_outer_waypoints(T_calculate, x_values, y_values, z_values, T_move):

    #Local variables
    new_left_side = 100
    new_right_side = -100
    new_y_cord = Get_lowest_point(T_calculate)
    even = 0
    
    while new_y_cord + (SCAN_HEAD / 4) <= Get_highest_point(T_calculate):
        even += 1
        """
        Finding the outer points
        """
        for cords_all in pcd.points:
            if cords_all[2] > T_calculate:
                if cords_all[0] > LS_WATER_CASE and cords_all[0] < RS_WATER_CASE: #check if the point is in the water case
                    if cords_all[1] >= new_y_cord and cords_all[1] < new_y_cord + 0.002:
                        # get the most left points
                        if cords_all[0] < new_left_side:
                            new_left_side = cords_all[0]
                        # get the most right points
                        if cords_all[0] > new_right_side:
                            new_right_side = cords_all[0]

        rounded_left_point = (new_left_side - 0.02)
        rounded_right_point = (new_right_side + 0.02)

        """
        this part adds the y cords to the array 2 times,
        this is done to because the x cord receives 2 values (left and right side)
        These are both on the same y cord
        """
        A = 0
        while A <= 1:
            rounded_new_Y = (new_y_cord - 0.01)
            y_values.append(rounded_new_Y)
            z_values.append(T_move)
            A += 1
        """
        switch the way arrays get filled for zigzag pattern
        """
        if even % 2 == 1:
            x_values.append(rounded_right_point) 
            Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even, T_calculate, z_values)
            x_values.append(rounded_left_point)
        elif even % 2 == 0:
            x_values.append(rounded_left_point)
            Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even, T_calculate, z_values)
            x_values.append(rounded_right_point) 
        new_y_cord += SCAN_HEAD

    return x_values, y_values

#Function for creating waypoint's between the outer points
def Create_inner_waypoints(x_values, y_values, rounded_left_point, rounded_new_Y, rounded_right_point, even, T_calculate, z_value):

    if even % 2 == 1:
        new_x_point = rounded_right_point
        while new_x_point > rounded_left_point:
            new_x_point = new_x_point - 0.01
            # z_value.append(Find_z(T_calculate, new_x_point, rounded_new_Y, z_value))
            z_value.append(0)
            x_values.append(new_x_point) 
            y_values.append(rounded_new_Y)
    elif even % 2 == 0:
        new_x_point = rounded_left_point
        while new_x_point < rounded_right_point:
            new_x_point = new_x_point + 0.01
            # z_value.append(Find_z(T_calculate, new_x_point, rounded_new_Y, z_value))
            z_value.append(0)
            x_values.append(new_x_point) 
            y_values.append(rounded_new_Y)

def Find_z(T_calculate, new_x_point, rounded_new_Y, z_value):
    z_avg = []
    for cords in pcd.points: #TODO find the reason why z value cannot be found for a smaller area
        if cords[2] > T_calculate and cords[0]>= new_x_point - SEARCH_AREA and cords[0] <= new_x_point + SEARCH_AREA and cords[1] >= rounded_new_Y - SEARCH_AREA and cords[1] <= rounded_new_Y + SEARCH_AREA:
            z_avg.append(cords[2])
    if not z_avg:
        avg = z_value[-1]
        return avg
    sum_z = sum(z_avg)
    avg = round(sum_z / len(z_avg), 4)
    return avg

#Function for ordering the waypoints in the right order
def fill_waypoints(x_values, y_values, wayPoints, z_values, rx, ry, rz):
    """
    ordering algorithm
    """          
    for h in range(len(x_values)):
        wayPoints.append([round(x_values[h], 4), round(y_values[h], 4), z_values[h], round(rx[h], 1), round(ry[h], 4), round(rz[h], 4)])

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

def Add_safety_points(wayPoints, safety_points):

    copy_list = copy.deepcopy(wayPoints)
    copy_list2 = copy.deepcopy(wayPoints)
    first_row = copy_list[0]
    last_row = copy_list[-1]
    lastlast_row = copy_list2[-1]
    safety_points.append(first_row)
    safety_points.append(last_row)
    safety_points.append(lastlast_row)
    safety_points[0][2] = 0
    safety_points[1][2] = -0.25
    safety_points[2][2] = 0

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
def Write_to_csv(wayPoints, safety_points, x_values, y_values):
    
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