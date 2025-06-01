#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Cámara Frontal - Catvehicle", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("Error al convertir la imagen: %s", e)

if __name__ == '__main__':
    rospy.init_node('visualizador_camara_catvehicle')
    bridge = CvBridge()

    rospy.Subscriber("/catvehicle/triclops/triclops/left/image", Image, callback)

    rospy.loginfo("🟢 Nodo visualizador de cámara frontal iniciado.")
    rospy.spin()
    cv2.destroyAllWindows()

