#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
import math
import os

def lidar_callback(msg):
    distances = []
    for point in msg.points:
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        distances.append(distance)
    
    # Define la ruta del archivo usando os.path.expanduser para manejar ~
    file_path = os.path.expanduser('~/catvehicle_ws/src/cmd_vel_listener/distances.txt')
    
    # Crea el directorio si no existe
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    
    # Guarda las distancias en un archivo de texto
    with open(file_path, 'w') as file:
        for distance in distances:
            file.write(f"{distance}\n")
    
    # Imprime el número total de puntos una vez
    total_points = len(distances)
    print(f"Total de puntos: {total_points}")

def listener():
    rospy.init_node('lidar_distance_listener', anonymous=True)
    rospy.Subscriber('/catvehicle/lidar_points', PointCloud, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


