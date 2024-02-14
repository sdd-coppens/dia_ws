#!/usr/bin/env python3
from custom_controller_interfaces.srv import VectorPredictionFK

import rclpy
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from rclpy.node import Node

import os.path
import pickle
import math


class PythonNode(Node):

    def __init__(self):
        super().__init__("calculate_vector")
        self.srv = self.create_service(VectorPredictionFK, "vector_prediction_fk", self.calculate_vector_fk)

        global rf_regressor
        model_pkl_file = "model_dump.pkl"

        if os.path.isfile("./model_dump.pkl"):
            with open(model_pkl_file, 'rb') as file:  
                rf_regressor = pickle.load(file)
        else:
            df = pd.read_csv("src/custom_controller/pos_controller/src/lookup_tables/angle_data_idk.csv")
            X = df.iloc[:, 3:6].values  # Selecting the last three columns as features
            y = df.iloc[:, 0:3].values  # Selecting the first three columns as the target

            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

            # Initialize the Random Forest Regressor
            # global rf_regressor
            rf_regressor = RandomForestRegressor(n_estimators=200, random_state=42)  # n_estimators is the number of trees

            # Fit the model to the training data
            rf_regressor.fit(X_train, y_train)

            # Predict on the test set
            y_pred = rf_regressor.predict(X_test)

            # Calculate the Mean Squared Error
            mse = mean_squared_error(y_test, y_pred)
            print(f"Mean Squared Error: {mse}")

            with open(model_pkl_file, 'wb') as file:  
                pickle.dump(rf_regressor, file)

        print("Done with setup.")


    def calculate_vector_fk(self, request, response):
        # TODO: figure out why order is different?
        new_motor_positions = np.array([request.pos_a, request.pos_c, request.pos_b]).reshape(1, -1)
        predicted_vector_values = rf_regressor.predict(new_motor_positions)
        response.nx = -float(predicted_vector_values[0][0])
        response.ny = float(predicted_vector_values[0][1])

        # TODO: no clue why this doesn't behave as expected.
        # response.nz = float(predicted_vector_values[0][2])
        response.nz = float(math.sqrt(1 - (response.nx*response.nx)-(response.ny*response.ny)))
        return response


def main(args=None):
    rclpy.init(args=args)

    python_node = PythonNode()

    rclpy.spin(python_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()