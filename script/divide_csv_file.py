#! /usr/bin/env python3
import math
import pandas as pd

"""
Execution method

STEP1:
~/catkin_ws/src/mobile_robot_simulator/script
STEP2:
python3 divide_csv_file.py 
"""
class DivideCsvFile():
    def __init__(self):
        # get pose data from csv file
        self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/kakunin_1106_left.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/kakunin_1106_right.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/mobile_robot_simulator/path/path_data_estimated_pose.csv")

        self.csv_file_number = len(self.csv_data) // 100
        self.csv_file_number_remainder = len(self.csv_data) % 100
        print(self.csv_file_number)
        print(self.csv_file_number_remainder)

        self.offset_path_num = 3

        # path dict for csvfile
        self.path_dict = {}

    def devide_csv_file(self):
        for csv_number in range(self.csv_file_number + 1):
            if csv_number == self.csv_file_number:
                for index in range(csv_number * 100 + self.offset_path_num, len(self.csv_data)):
                    position_x = self.csv_data["x"][index]
                    position_y = self.csv_data["y"][index]
                    position_z = self.csv_data["z"][index]
                    orientation_x = self.csv_data["w0"][index]
                    orientation_y = self.csv_data["w1"][index]
                    orientation_z = self.csv_data["w2"][index]
                    orientation_w = self.csv_data["w3"][index]

                    current_point = [position_x,
                                     position_y,
                                     position_z,
                                     orientation_x,
                                     orientation_y,
                                     orientation_z,
                                     orientation_w,
                                    ]

                    self.path_dict[len(self.path_dict)] = current_point
            elif csv_number == 0:
                for index in range(csv_number * 100, csv_number * 100 + 100):
                    position_x = self.csv_data["x"][index]
                    position_y = self.csv_data["y"][index]
                    position_z = self.csv_data["z"][index]
                    orientation_x = self.csv_data["w0"][index]
                    orientation_y = self.csv_data["w1"][index]
                    orientation_z = self.csv_data["w2"][index]
                    orientation_w = self.csv_data["w3"][index]

                    current_point = [position_x,
                                     position_y,
                                     position_z,
                                     orientation_x,
                                     orientation_y,
                                     orientation_z,
                                     orientation_w,
                                    ]

                    self.path_dict[len(self.path_dict)] = current_point
            else:
                for index in range(csv_number * 100 + self.offset_path_num, csv_number * 100 + 100):
                    position_x = self.csv_data["x"][index]
                    position_y = self.csv_data["y"][index]
                    position_z = self.csv_data["z"][index]
                    orientation_x = self.csv_data["w0"][index]
                    orientation_y = self.csv_data["w1"][index]
                    orientation_z = self.csv_data["w2"][index]
                    orientation_w = self.csv_data["w3"][index]

                    current_point = [position_x,
                                     position_y,
                                     position_z,
                                     orientation_x,
                                     orientation_y,
                                     orientation_z,
                                     orientation_w,
                                    ]

                    self.path_dict[len(self.path_dict)] = current_point

            # reset dict
            Divide_csv_file.save_csv(csv_number)
            self.path_dict = {}

    def save_csv(self, csv_number):
        # Save CSV path file
        cols = ["x", "y", "z", "w0", "w1", "w2", "w3"]
        file_name = "test_course" + str(csv_number + 1) + ".csv"
        #file_path = "~/catkin_ws/src/mobile_robot_simulator/path/" + file_name
        file_path = "~/catkin_ws/src/mobile_robot_simulator/path/tsukuba_kakunin_left_path/" + file_name
        #file_path = "~/catkin_ws/src/mobile_robot_simulator/path/tsukuba_kakunin_right_path/" + file_name
        #file_path = "~/catkin_ws/src/mobile_robot_simulator/path/hirayama_path/" + file_name

        df = pd.DataFrame.from_dict(self.path_dict, orient='index',columns=cols)
        df.to_csv(file_path, index=False)

if __name__ == "__main__":
    Divide_csv_file = DivideCsvFile()
    Divide_csv_file.devide_csv_file()