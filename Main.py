#######################################
## Code for running the whole system ##
#######################################

from os.path import exists
import os
# import Testpad
import PadGen2D
import CameraHandling
from testSending import *
import time
import keyboard

allKey = 'a'
cobotMoveKey = 'm'
scanKey = 's'
breakKey = 'b'

def Start_message():
    print("Program started")

def visionScan():
    Start_message()

    # os.system("CameraHandling.py")
    CameraHandling.main()

    time.sleep(6)

    #check if the point cloud exists
    file_exists = exists("test.ply")
    if file_exists:
        print("executing 2d path generation")
        #start the pathgenerator
        # Testpad.main()
        PadGen2D.main()  
    else:
        print("Please check if the camera is connected and the program is running")
        exit()

def startMove(robotClass):
    #check if the path exists
    file_exists = exists("cords.csv")
    if file_exists:
        #start the pathfollower
        robotClass.mainCobotScript()

        print("Path found")
    else:
        print("No path found")
        print("Please check if the camera is connected and the program is running")
        exit()

def main():
     # Start the robot
    robot = RTDE_urx()
    test = KeepAlive(robot.con, robot.watchdog)
    test.start()
    print("Robot started")


    while True:  # making a loopm
        
        try:  # used try so that if user pressed other than the given key error will not be shown
            if keyboard.is_pressed(allKey):  # if key 'all' is pressed 
                print(f'You Pressed the {allKey} Key!')
                os.remove("test.ply") 
                os.remove("cords.csv")  
                visionScan() # starts the 3d path generation of the product
                startMove(robot) # Makes the cobot follow the generated path
                time.sleep(0.5)
                continue

            elif keyboard.is_pressed(scanKey): # if key 'scan' is pressed
                print(f'You Pressed the {scanKey} Key!')
                visionScan() # starts the 3d path generation of the product
                time.sleep(0.5)
                continue

            elif keyboard.is_pressed(cobotMoveKey): # if key 'move' is pressed
                print(f'You Pressed the {cobotMoveKey} Key!')
                startMove(robot) # Makes the cobot follow the generated path
                time.sleep(0.5)
                continue
        except:
            break  # if user pressed a key other than the given key the loop will break
    
    

    # #delete the previous created files 

if __name__ == '__main__':
    main()
