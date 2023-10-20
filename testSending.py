# This is adapted code that originates from:
# https://github.com/LaurentBimont/RTDE-URx/tree/master
# It was downloaded/adapted on the 17th of april

import sys
sys.path.append('..')
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import numpy as np
from threading import Thread
import time
import csv

# Hier worden de locaties naar geupload
tempLocation = [0,0,0,0,0,0]
feature_wrt_base = []

class RTDE_urx(object):


    def __init__(self, ROBOT_HOST='192.168.0.1',
                 ROBOT_PORT=30004,
                 config_filename='control_loop_configuration.xml'):

        keep_running = True
        logging.getLogger().setLevel(logging.INFO)
        conf = rtde_config.ConfigFile(config_filename)

        state_names, state_types = conf.get_recipe('state')
        setp_names, setp_types = conf.get_recipe('setp')
        watchdog_names, watchdog_types = conf.get_recipe('watchdog')
        gripper_names, gripper_types = conf.get_recipe("gripper")

        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
        self.gripper = self.con.send_input_setup(gripper_names, gripper_types)
        ### Initialize variables of the register
        self.initialize()

        # start data synchronization
        if not self.con.send_start():
            sys.exit()

    def initialize(self):
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0
        self.gripper.input_int_register_1 = 0
        self.watchdog.input_int_register_0 = 0
        self.setp.input_int_register_25 = 1
        # self.con.send(self.setp)

    def setp_to_list(self):
        list = []
        for i in range(0, 6):
            list.append(self.setp.__dict__["input_double_register_%i" % i])
        return list

    def list_to_setp(self, list):
        for i in range(0, 6):
            self.setp.__dict__["input_double_register_%i" % i] = list[i]
        return self.setp

    def isMoveReady(self):
        ''''

        If the cobot has arrived at the desired waypoint, it will set a int variable to 1.
        The RTDE code can interpret this as a sign to send a new waypoint.

        '''
        state = self.con.receive()
        if state.output_int_register_24 == 1:
            # print("The cobot arrived at waypoint")
            return True
        else:
            print("The cobot has not arrived at waypoint")
            return False

    def movel(self, pos):
        '''
        Perform a linear move to cartesian position pos
        :param pos: cartesian position (x, y, z, Rx, Ry, Rz)
        :return: None, break when the actual position is <2mm from the target position
        '''
        print('target position', pos)
        while True:
            print("Move is not ready")
            print(pos)
            
            # print('Location send to cobot')
            if self.isMoveReady():
                print("Move is ready")
                self.list_to_setp(pos)
                self.con.send(self.setp)
                print('Arrived at pos')
                break

    def mainCobotScript(self):
        coord_list = read_csv_coords('cords.csv')
        print(coord_list)

        self.setp.input_int_register_25 = 1

        # self.movel([0,0,0,0,0,0])
        for coord in coord_list:
            print("Next position")
            print(coord)
            self.movel(coord)
            # time.sleep(5)
            continue
        
        # self.state.input_int_register_24 = 0
        # self.movel([0,0,0,0,0,0])
        # self.con.send(self.setp)



        # time.sleep(10)
        while not self.isMoveReady():
            pass

        self.setp.input_int_register_25 = 0
        # self.setp.input_int_register_24 = 0
        # while not self.isMoveReady():  
        #     self.movel([0,0,0,0,0,0])
        self.con.send(self.setp)




        # state = self.con.receive()
        # print(state.output_int_register_24, "state")
        # self.con.send(self.setp)
        # if self.con.input_int_register_25 == 0:
        #     print("ik ben ronan en ik kan niet coderen)")
        #     # return True
        print("kom ik hier")




class KeepAlive(Thread):
    def __init__(self, con, watchdog):
        super().__init__()
        self.con = con
        self.watchdog = watchdog
        self.daemon = True

    def run(self):
        while True:
            state = self.con.receive()
            self.con.send(self.watchdog)
            time.sleep(0.1)

def read_csv_coords(csv_file_param):
    coord_list = []
    with open(csv_file_param) as csv_file:
        file = csv.reader(csv_file)
        for row in file:

            # One line for coord, all in string
            coord_txt = row[0].split(';')

            # fillable list for converting to float
            coord_list_fillable = []

            if coord_txt == ['x', 'y', 'z', 'rx', 'ry', 'rz']:
                continue
            else:
                counter = 0
                for val in coord_txt:
                    if counter in [0,1,2,3,4,5]:
                        coord_list_fillable.append(float(val))


            coord_list.append(coord_list_fillable)

        return coord_list


if __name__== "__main__":
    robot = RTDE_urx()
    test = KeepAlive(robot.con, robot.watchdog)
    test.start()
    
    robot.mainCobotScript()

    # List of coordinates
    # coord_list = read_csv_coords('cords.csv')
    # Coord structure
    # [x, y, z, rX, rY, rZ]

    # while True:
    #     print(coord_list)
    #     robot.movel([0,0,0,0,0,0])
    #     for coord in coord_list:
    #         print("Next position")


    #         robot.movel(coord)
    #         # time.sleep(5)
    #         continue

    #     robot.movel([0,0,0,0,0,0])
    #     robot.setp.input_int_register_25 = 0
    #     print("kom ik hier?")
    #     break

