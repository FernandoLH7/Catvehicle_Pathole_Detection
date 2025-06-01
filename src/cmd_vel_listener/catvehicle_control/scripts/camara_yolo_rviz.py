#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import torch
from ultralytics import YOLO

bridge = CvBridge()

# Cargar el modelo YOLOv8n
model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "yolov8n.pt")
model = YOLO(model_path)

# Verifica si tienes GPU
device = "cuda" if torch.cuda.is_available() else "cpu"
rospy.loginfo(f"🧠 YOLOv8 está usando: {device.upper()}")

# Publisher para RViz
image_pub = None

def callback(msg):
    global image_pub
    try:
        # Convertir la imagen ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        resized = cv2.resize(frame, (480, 480))

        # Inferencia
        results = model.predict(source=resized, show=False, conf=0.3, verbose=False, device=device)[0]

        # Dibujar detecciones
        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = results.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(resized, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publicar resultado en RViz
        annotated_msg = bridge.cv2_to_imgmsg(resized, encoding="bgr8")
        annotated_msg.header = Header()
        annotated_msg.header.stamp = rospy.Time.now()
        image_pub.publish(annotated_msg)

    except CvBridgeError as e:
        rospy.logerr(f"Error de conversión: {e}")
    except Exception as e:
        rospy.logerr(f"Error en inferencia: {e}")

if __name__ == '__main__':
    rospy.init_node('camara_yolov8_rviz')
    rospy.loginfo("🟢 Nodo con YOLOv8 publicando a /inference_result para RViz")

    image_pub = rospy.Publisher("/inference_result", Image, queue_size=1)
    rospy.Subscriber("/catvehicle/triclops/triclops/left/image", Image, callback)

    rospy.spin()

