#!/usr/bin/env python3
import rospy
import socket
import struct
import pickle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HOST = "10.41.146.149"
PORT = 5000

rospy.init_node("camera_bridge", anonymous=True)
pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
bridge = CvBridge()

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

data = b""
payload_size = struct.calcsize("!L")

while not rospy.is_shutdown():
    while len(data) < payload_size:
        data += client_socket.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("!L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += client_socket.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data)
    ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub.publish(ros_image)
