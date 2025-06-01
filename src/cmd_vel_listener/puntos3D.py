#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
import math
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def lidar_callback(msg):
    distances = []
    x_coords = []
    y_coords = []
    z_coords = []
    colors = []

    # Define los colores para las 5 secciones
    section_colors = ['red', 'blue', 'green', 'yellow', 'orange']

    for i, point in enumerate(msg.points[:2000]):
        # Calcular la distancia
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        distances.append(distance)

        # Agregar coordenadas x, y, z a las listas
        x_coords.append(point.x)
        y_coords.append(point.y)
        z_coords.append(point.z)

        # Calcular la sección en la que se encuentra el punto
        section = (i % 100) // 20
        colors.append(section_colors[section])
    
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

    # Visualizar los puntos en un gráfico 3D
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_coords, y_coords, z_coords, c=colors, marker='o')
    ax.set_title('Distribución de los puntos LIDAR en el espacio 3D')
    ax.set_xlabel('X (metros)')
    ax.set_ylabel('Y (metros)')
    ax.set_zlabel('Z (metros)')
    plt.show()

def listener():
    rospy.init_node('lidar_distance_listener', anonymous=True)
    rospy.Subscriber('/catvehicle/lidar_points', PointCloud, lidar_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

