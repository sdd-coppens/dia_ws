#!/usr/bin/env python3
from custom_controller_interfaces.srv import VectorPredictionFKTwo
from std_msgs.msg import String

import rclpy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from sklearn.preprocessing import PolynomialFeatures
from rclpy.node import Node

import os.path
import pickle
import math
import time


class PythonNode(Node):

    def __init__(self):
        super().__init__("calculate_vector")
        # self.srv = self.create_service(VectorPredictionFKTwo, "/vector_prediction_fk_two", self.calculate_vector_fk)
        self.srv = self.create_service(VectorPredictionFKTwo, "/vector_prediction_fk_two", self.calculate_vector_fk_two)
        self.publisher_sync_ = self.create_publisher(String, "/sync_signal", 10)

        global rf_regressor
        global poly_reg_model
        global poly
        model_pkl_file = "model_dump4.pkl"

        if os.path.isfile("./model_dump4.pkl"):
            with open(model_pkl_file, 'rb') as file:  
                rf_regressor = pickle.load(file)
        else:
            df = pd.read_csv("ski_testing3.csv")
            X = df.iloc[:, 4:7].values
            y = df.iloc[:, 0:4].values
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

            if False:            
                rf_regressor = RandomForestRegressor(n_estimators=1, random_state=0)
                rf_regressor.fit(X_train, y_train)
                y_pred = rf_regressor.predict(X_test)
                mse = mean_squared_error(y_test, y_pred)
                self.get_logger().info(f"Mean Squared Error: {mse}")
                with open(model_pkl_file, 'wb') as file:  
                    pickle.dump(rf_regressor, file)
            else:
                degree = 1
                poly = PolynomialFeatures(degree=degree)
                X_train_poly = poly.fit_transform(X_train)
                X_test_poly = poly.transform(X_test)

                poly_reg_model = LinearRegression()
                poly_reg_model.fit(X_train_poly, y_train)
                
                test_score = poly_reg_model.score(X_test_poly, y_test)
                self.get_logger().info(f"R² Score on Test Data: {test_score}")


        self.get_logger().info("Setup done.")
        msg = String()
        msg.data = "FK Start"
        self.publisher_sync_.publish(msg)

# 0.195999, 0.195999, 0.960817, 580, 162, 451
        new_motor_positions = np.array([253, 704, 356]).reshape(1, -1)
        print(new_motor_positions)
        predicted_vector_values = []
        if False:
            predicted_vector_values = rf_regressor.predict(new_motor_positions)
        else:
            new_motor_positions_poly = poly.fit_transform(new_motor_positions)
            predicted_vector_values = poly_reg_model.predict(new_motor_positions_poly)
        # 
        print(predicted_vector_values)

    def calculate_vector_fk(self, request, response):
        # TODO: figure out why order is different?
        new_motor_positions = np.array([request.pos_a, request.pos_c, request.pos_b]).reshape(1, -1)
        new_motor_positions = np.array([request.pos_a, request.pos_b, request.pos_c]).reshape(1, -1)
        predicted_vector_values = rf_regressor.predict(new_motor_positions)
        response.qx = float(predicted_vector_values[0][0])
        response.qy = float(predicted_vector_values[0][1])
        response.qz = float(predicted_vector_values[0][2])
        response.qw = float(1)#float(predicted_vector_values[0][3])
        return response

    def calculate_vector_fk_two(self, request, response):
        # TODO: figure out why order is different?
        new_motor_positions = np.array([request.pos_a, request.pos_c, request.pos_b]).reshape(1, -1)
        new_motor_positions = np.array([request.pos_a, request.pos_b, request.pos_c]).reshape(1, -1)
        new_motor_positions_poly = poly.fit_transform(new_motor_positions)
        predicted_vector_values = poly_reg_model.predict(new_motor_positions_poly)
        response.qx = float(predicted_vector_values[0][0])
        response.qy = float(predicted_vector_values[0][1])
        response.qz = float(predicted_vector_values[0][2])
        response.qw = float(1)#float(predicted_vector_values[0][3])
        return response


def main(args=None):
    rclpy.init(args=args)

    python_node = PythonNode()

    rclpy.spin(python_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()