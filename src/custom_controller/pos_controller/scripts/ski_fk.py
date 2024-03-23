#!/usr/bin/env python3
from custom_controller_interfaces.srv import VectorPredictionFK
from std_msgs.msg import String

import rclpy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import PolynomialFeatures
from rclpy.node import Node

import os.path
import pickle
import math
import time


class FKNode(Node):

    def __init__(self):
        super().__init__("calculate_vector")
        self.srv = self.create_service(VectorPredictionFK, "/vector_prediction_fk", self.calculate_vector_fk)
        self.publisher_sync_ = self.create_publisher(String, "/sync_signal", 10)

        global rf_regressor
        model_pkl_file = "model_dump2.pkl"

        if os.path.isfile("./model_dump2.pkl"):
            with open(model_pkl_file, 'rb') as file:  
                rf_regressor = pickle.load(file)
        else:
            df = pd.read_csv("ski_testing.csv")
            X = df.iloc[:, 4:7].values
            y = df.iloc[:, 0:4].values

            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

            degree = 5
            poly = PolynomialFeatures(degree=degree)

            X_train_poly = poly.fit_transform(X_train)
            X_test_poly = poly.transform(X_test)

            poly_reg_model = LinearRegression()
            poly_reg_model.fit(X_train_poly, y_train)
            
            test_score = poly_reg_model.score(X_test_poly, y_test)
            self.get_logger().info(f"R² Score on Test Data: {test_score}")

            # with open(model_pkl_file, 'wb') as file:  
                # pickle.dump(poly_reg_model, file)

        self.get_logger().info("Setup done.")
        msg = String()
        msg.data = "FK Start"
        self.publisher_sync_.publish(msg)


# 0.195999, 0.195999, 0.960817, 580, 162, 451
        new_motor_positions = np.array([580, 162, 451]).reshape(1, -1)
        print(new_motor_positions)
        # predicted_vector_values = rf_regressor.predict(new_motor_positions)
        # print(predicted_vector_values)




    def calculate_vector_fk_new_test(self, request, response):
        blabla = 1
        # import csv
        # csv_file = "src/custom_controller/pos_controller/src/lookup_tables/angle_data_no_quaternion.csv"
        # last_three_values = [str(request.pos_a), str(request.pos_c), str(request.pos_b)]
        # print(last_three_values)
        # with open(csv_file, "r") as file:
        #     reader = csv.reader(file)
        #     for row in reader:
        #         # Check if the last three values in the row match the input
                
        #         if row[-3:] == last_three_values:
        #             print(row[:3])






    def calculate_vector_fk(self, request, response):
        blabla = 1
        # # self.calculate_vector_fk_new_test(request, response)

        # # TODO: figure out why order is different?
        # new_motor_positions = np.array([request.pos_a, request.pos_c, request.pos_b]).reshape(1, -1)


        # new_motor_positions = np.array([request.pos_a, request.pos_b, request.pos_c]).reshape(1, -1)
        # # now = time.time_ns()
        # # print(new_motor_positions)
        # predicted_vector_values = rf_regressor.predict(new_motor_positions)
        # # print(predicted_vector_values)
        # # print("time to predcit vector values: {}".format((time.time_ns() - now) / 1000000))
        # response.nx = -float(predicted_vector_values[0][0])
        # response.ny = float(predicted_vector_values[0][1])

        # # TODO: no clue why this doesn't behave as expected.
        # # response.nz = float(predicted_vector_values[0][2])
        # response.nz = float(math.sqrt(1 - (response.nx*response.nx)-(response.ny*response.ny)))
        # response.pos_x_axis = request.pos_x_axis
        # return response


def main(args=None):
    rclpy.init(args=args)

    python_node = FKNode()

    rclpy.spin(python_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()