# open3d and padgen imports
import numpy as np
import open3d as o3d
# camera imports
import cv2
import pyrealsense2
import csv
import heapq

def main():
    pcd = o3d.io.read_point_cloud("out.ply")

    counter = 0
    highest_point_z = -100
    most_left_point = 100
    most_right_point = -100
    highest_point_y = -100
    lowest_point_y = 100
    # array for x values
    x_values = []
    # array for y values
    y_values = []
    # array for z calculations
    z_values = []
    # print the amount of points in the point cloud
    print(pcd)

    calc_points_topPlane = 500
    top_safety = 0.02
    top_plane_readArea = 0.1

############################################################
    #TODO make a function that prevents the algorithm running over waterbakd inside of in it


############################################################    
    for cords_all in pcd.points:
        z_values.append(cords_all[2])
    sorted_z_values = heapq.nlargest(calc_points_topPlane, z_values)
    #add all the values from the array
    total_z_value = sum(sorted_z_values)
    #divide the total by the amount of values
    average_z_value = total_z_value / len(sorted_z_values)
    top = average_z_value + top_safety
    top_plane = round(average_z_value - top_plane_readArea, 4)
    print(average_z_value, "sorted z values")
    print(top, "top")
############################################################
# not needed anymore
    # use this to find the top value you need for top
    # for cords_all in pcd.points:
    #     if cords_all[2] > highest_point_z:
    #         highest_point_z = cords_all[2]
    # print(highest_point_z, "highest point")
############################################################

    # print all cords with the z value higher than -0.3
    for cords_all in pcd.points:
        if cords_all[2] > top_plane:
            if cords_all[0] < most_left_point:
                most_left_point = cords_all[0]
            if cords_all[0] > most_right_point:
                most_right_point = cords_all[0]
            if cords_all[1] > highest_point_y:
                highest_point_y = cords_all[1]
            if cords_all[1] < lowest_point_y:
                lowest_point_y = cords_all[1]
            # print(cords_all)
            counter += 1
    print(counter, "counter")
    print(most_left_point, "most left point")
    print(most_right_point, "most right point")
    print(highest_point_y, "highest point y")
    print(lowest_point_y, "lowest point y")

    # print the cord with the most left x value and the highest y value
    for cords_all in pcd.points:
        if cords_all[0] == most_left_point:
            if cords_all[1] == highest_point_z:
                print(cords_all, "most left and highest point")

    for cords_all in pcd.points:
        if cords_all[0] == most_left_point:
            print(cords_all, "most left point")
    # print the cord with the most right x value
    for cords_all in pcd.points:
        if cords_all[0] == most_right_point:
            print(cords_all, "most right point")

    for cords_all in pcd.points:
        if cords_all[1] == highest_point_y:
            print(cords_all, "highest point y")

    for cords_all in pcd.points:
        if cords_all[1] == lowest_point_y:
            print(cords_all, "lowest point y")
############################################################
    # check left side and right side based on the scanhead size
    new_Lowest_point_y = lowest_point_y
    new_left_side = 100
    new_right_point = -100

    # print(lowest_point_y, "lowest point y ---test---")
    # print(highest_point_y, "highest point y ---test---")
    while new_Lowest_point_y < highest_point_y:
        new_Lowest_point_y += 0.046
        new_left_side = 100
        J = 0
        # print(new_Lowest_point_y, "new lowest point y ---test---")
        while J <= 1:
            rounded_new_lowest_point_y = round(new_Lowest_point_y, 4)
            y_values.append(rounded_new_lowest_point_y)
            J += 1

        for cords_all in pcd.points:
            if cords_all[1] >= new_Lowest_point_y and cords_all[1] < new_Lowest_point_y + 0.002:
                # get the most left points
                if cords_all[0] < new_left_side:
                    new_left_side = cords_all[0]
                # get the most right points
                if cords_all[0] > new_right_point:
                    new_right_point = cords_all[0]
        rounded_new_left_point = round(new_left_side, 4)
        rounded_new_right_point = round(new_right_point, 4)
        x_values.append(rounded_new_right_point)
        x_values.append(rounded_new_left_point)
        # print(new_left_side, "new left side")
        # print(new_right_point, "new right point")
    print(x_values, "x values")
    print(len(x_values), "x values length")
    print(y_values, "y values")
    print(len(y_values), "y values length")
############################################################
    # collect the cords in a list
    header = ["x", "y", "z", "rx", "yx", "zx"]
    data_bot = [[round(most_left_point, 4), round(lowest_point_y, 4), top_plane, 0, 0, 20],
                [round(most_right_point, 4), round(lowest_point_y, 4), top_plane, 0, 0, 20]]
    data_top = [[round(most_left_point, 4), round(highest_point_y, 4), top_plane, 0, 0, 20],
               [round(most_right_point, 4), round(highest_point_y, 4), top_plane, 0, 0, 20]]

    # write the cords to a csv file
    with open('cords.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=";")
        writer.writerow(header)
        writer.writerows(data_bot)
        data2 = []
        for h in range(len(x_values)):
            if h < 2:
                data2.append([round(x_values[h], 4), round(y_values[h], 4), top_plane, 0, 0, 0])
            elif h >=2 and (h % 2) == 1 and h != len(x_values) - 1:
                h = h + 1
                data2.append([round(x_values[h], 4), round(y_values[h], 4), top_plane, 0, 0, 0])
            elif h >=2 and (h % 2) == 0:
                h = h - 1
                data2.append([round(x_values[h], 4), round(y_values[h], 4), top_plane, 0, 0, 0])
            # writer.writerow(data2.pop(0))
        writer.writerows(data2)
        writer.writerows(data_top)

if __name__ == "__main__":
    main()
