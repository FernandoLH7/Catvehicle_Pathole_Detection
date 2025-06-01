#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
import math

def lidar_callback(msg):
    for point in msg.points:
        # Calcular la distancia
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        if distance < 49:
            print(f"Distancia detectada: {distance}")

def listener():
    rospy.init_node('lidar_distance_listener', anonymous=True)
    rospy.Subscriber('/catvehicle/lidar_points', PointCloud, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


