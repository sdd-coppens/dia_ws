#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import io
from custom_controller_interfaces.msg import ArduinoMotor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
import os.path


def quaternion_rotation(quaternion, point):
    # Unpack quaternion and point
    w, x, y, z = quaternion
    px, py, pz = point

    # Normalize quaternion
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    # Quaternion multiplication (point as quaternion with w=0)
    # Result of rotation: q * p * q^-1
    # Since q is unit quaternion, q^-1 = q*. Therefore, the operation becomes q * p * q*

    # q * p
    qw, qx, qy, qz = w, x, y, z
    pw, px, py, pz = 0, px, py, pz
    rx =  qw*px + qy*pz - qz*py
    ry =  qw*py + qz*px - qx*pz
    rz =  qw*pz + qx*py - qy*px
    rw = -qx*px - qy*py - qz*pz

    # (q * p) * q*
    nx = rw*-qx + rx*qw - ry*qz + rz*qy
    ny = rw*-qy + ry*qw - rz*qx + rx*qz
    nz = rw*-qz + rz*qw - rx*qy + ry*qx

    return nx, ny, nz

    # Define quaternion and point


class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        self.publisher_ = self.create_publisher(ArduinoMotor, "/arduino_motor_serial", 10)
        self.publisher_pose_ = self.create_publisher(PoseStamped, "/object/pose_non_network", 10)
        self.disconnected = False

        # Check if arduino is attached.
        while True:
            try:
                self.serial_port = serial.Serial("/dev/ttyACM0", 115200, timeout=0.001)
                self.get_logger().info("arduino found.")
                self.disconnected = False
                break
            except:
                if not self.disconnected:
                    self.disconnected = True
                    self.get_logger().info("no arduino found.")
                    time.sleep(1)

        self.timer = self.create_timer(0.001, self.timer_callback)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.serial_port, self.serial_port))
        self.keyboard_bool = False

        self.sio.write("start\n")
        self.sio.flush()

        self.subscription = self.create_subscription(String, "/keyboard", self.keyboard_listener_callback, 10)

    # Arduino disconnected, reconnect it.
    def reconnect_arduino(self):
        self.serial_port.close()
        try:
            self.serial_port = serial.Serial("/dev/ttyACM0", 115200, timeout=0.001)
            self.get_logger().info("arduino found.")
            self.sio = io.TextIOWrapper(io.BufferedRWPair(self.serial_port, self.serial_port))
            self.sio.write("start\n")
            self.sio.flush()
            self.disconnected = False
        except:
            if not self.disconnected:
                self.disconnected = True
                self.get_logger().info("no arduino found.")
            return

    def keyboard_listener_callback(self, msg):
        try:
            sending = False
            if msg.data == "f":
                if self.keyboard_bool:
                    self.sio.write("0.196078,0.196078,0.0\n")
                    # self.sio.write("0.196078,0.0,0.0\n")
                    self.keyboard_bool = False
                    # print("0.196078,0.196078,0.0\n")
                else:
                    self.sio.write("-0.196078,-0.196078,0.0\n")
                    # self.sio.write("-0.196078,0.0,0.0\n")
                    self.keyboard_bool = True
                    # print("-0.196078,-0.196078,0.0\n")
                sending = True
            elif msg.data == "k":
                self.sio.write("0.0,0.0,0.0\n")
                sending = True

            if sending:
                self.sio.flush()
            self.first_rec = True
        except:
            self.reconnect_arduino()
            # keyboard_listener_callback(self, msg)

    def timer_callback(self):
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("utf-8").rstrip()
                line_split = line.split(",")
                msg = ArduinoMotor()
                msg.timestamp = int(line_split[0])
                msg.pos_a = int(line_split[1])
                msg.pos_b = int(line_split[2])
                msg.pos_c = int(line_split[3])
                msg.x_pos = int(line_split[4])
                # self.get_logger().info(f"msg: {msg.pos_a}, {msg.pos_b}, {msg.pos_c}")
                # print(f"msg: {msg.pos_a}, {msg.pos_b}, {msg.pos_c}")
                self.publisher_.publish(msg)
        except:
            self.reconnect_arduino()

def main(args=None):
    quaternion = (1.32348, -0.0990148, 0, -0.0990146)
    quaternion2 = (1, -0.1, 0, -0.1)
    point = (0.935, 0, 0.705)
    quaternion_rotation(quaternion, point)
    quaternion_rotation(quaternion2, point)

    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()