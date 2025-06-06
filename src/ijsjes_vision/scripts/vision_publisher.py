#!/usr/bin/env python
import rospy
import socket
import json
from ijsjes_robot.msg import IceTarget 

def main():
    rospy.init_node('vision_publisher_node')  # Initialiseer ROS-node
    pub = rospy.Publisher('ice_target', IceTarget, queue_size=10)  # Publisher op ROS-topic

    # Mapping van herkende labels naar bijbehorende baknummers
    label_to_bin = {
        "knetterijsje": 2,
        "magnum": 3,
        "nogger": 4,
        "raketje": 6,
        "regenboogijsje": 5,
        "watermeloen": 1
    }

    # TCP-server opzetten voor communicatie met vision_control.py (Python 3-script)
    host = 'localhost'
    port = 9999
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(1)
    rospy.loginfo("Wacht op verbinding van vision_control.py...")

    # Verbinding accepteren
    conn, addr = s.accept()
    rospy.loginfo("Verbonden met vision_control op {}.".format(addr))

    while not rospy.is_shutdown():
        data = conn.recv(1024)  # Ontvang JSON-data via socket
        if not data:
            continue

        try:
            # JSON decoderen en vertalen naar ROS message
            info = json.loads(data)
            label = info["label"]
            if label in label_to_bin:
                msg = IceTarget()
                msg.x = info["x"]
                msg.y = info["y"]
                msg.z = info["z"]
                msg.rz = info["rz"]
                msg.bin_number = label_to_bin[label]
                pub.publish(msg)  # Verstuur message via ROS-topic
        except Exception as e:
            rospy.logwarn("Fout bij verwerken data: {}".format(e))

    # Verbindingen netjes afsluiten
    conn.close()
    s.close()

if __name__ == "__main__":
    main()
