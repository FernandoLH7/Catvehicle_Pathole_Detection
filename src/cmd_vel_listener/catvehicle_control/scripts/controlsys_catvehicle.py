#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist
import math

class ControlSysCatvehicle:
    def __init__(self):
        rospy.init_node('controlsys_catvehicle', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/catvehicle/lidar_points', PointCloud, self.lidar_callback)
        self.twist = Twist()
        self.twist.linear.x = 3.0  # velocidad inicial
        self.twist.angular.z = 0.0

    def lidar_callback(self, msg):
        closest_distance = float('inf')
        valid_points = 0

        for point in msg.points:
            distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)

            # Filtrar puntos inválidos
            if distance > 0.1 and distance < 100:  # Ignorar basura o puntos congelados
                valid_points += 1
                if distance < closest_distance:
                    closest_distance = distance

        rospy.loginfo(f"🔎 Puntos válidos: {valid_points}, Distancia más cercana: {closest_distance:.2f}")

        if valid_points == 0:
            rospy.logwarn("⚠️ No se detectaron puntos válidos. Continuando...")
            self.twist.linear.x = 3.0
        elif closest_distance < 15.0:
            rospy.loginfo(f"⚠️ Objeto detectado a {closest_distance:.2f} unidades. Deteniendo vehículo.")
            self.twist.linear.x = 0.0
        else:
            self.twist.linear.x = 3.0

        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        control = ControlSysCatvehicle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

