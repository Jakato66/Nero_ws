#!/usr/bin/env python3
import rclpy
import os
import sys
import threading
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray

"""This node handle the inputs and show the log outputs of the simulation in a window"""

# class InterfaceNode(Node):
#     def __init__(self):
# Este nodo puede que no sea necesario ya que existe rqt_console y rqt_publisher
