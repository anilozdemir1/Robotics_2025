#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from ijsjes_robot.msg import IceTarget  # ← jouw custom message
from geometry_msgs.msg import Pose
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
import tf
import geometry_msgs.msg
import math

class IceRobot:
    def __init__(self):
        rospy.init_node('robot_control_node')
        moveit_commander.roscpp_initialize([])

        # Wacht tot MoveIt (move_group) actief is
        rospy.loginfo("Wachten op move_group server...")
        rospy.wait_for_service('/get_planning_scene')
        rospy.sleep(2)

        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("endeffector")

        rospy.loginfo("Planning frame arm: {}".format(self.arm.get_planning_frame()))
        rospy.loginfo("Planning frame gripper: {}".format(self.gripper.get_planning_frame()))
        rospy.loginfo("End effector link arm: {}".format(self.arm.get_end_effector_link()))
        rospy.loginfo("End effector link gripper: {}".format(self.gripper.get_end_effector_link()))

        # TF listener voor transformaties
        self.tf_listener = tf.TransformListener()

        # Blijf wachten tot gripper-driver topic actief is
        rospy.loginfo("Wachten op gripper driver topic '/Robotiq2FGripperRobotInput'...")
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message("/Robotiq2FGripperRobotInput", rospy.AnyMsg, timeout=2)
                rospy.loginfo("Gripper driver is actief.")
                break
            except rospy.ROSException:
                rospy.logwarn("Gripper driver nog niet beschikbaar, opnieuw proberen...")
                rospy.sleep(1)

        # Publisher om gripper fysiek aan te sturen
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)
        rospy.sleep(1)

        # Activeer en reset de gripper
        self.gripper_activate_and_reset()

        # Zet gripper in open stand, visueel én fysiek
        self.move_gripper("open")

        # Home positie
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

        # Subscriben op ijsje target data
        rospy.Subscriber("/ice_target", IceTarget, self.target_callback)
        rospy.loginfo("Wacht op nieuwe ijs-coördinaten...")

    def gripper_activate_and_reset(self):
        cmd = Robotiq2FGripper_robot_output()
        # Activate
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rSP = 255  # snelheid
        cmd.rFR = 150  # kracht
        self.gripper_pub.publish(cmd)
        rospy.sleep(1)
        # Reset
        cmd.rACT = 0
        self.gripper_pub.publish(cmd)
        rospy.sleep(1)
        # Activate opnieuw
        cmd.rACT = 1
        self.gripper_pub.publish(cmd)
        rospy.sleep(1)

    def move_gripper(self, action):
        cmd = Robotiq2FGripper_robot_output()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rSP = 255
        cmd.rFR = 150

        if action == "open":
            cmd.rPR = 0
            self.gripper.set_named_target("open")
        elif action == "close":
            cmd.rPR = 255
            self.gripper.set_named_target("closed")
        else:
            rospy.logwarn("Onbekende gripper actie: {}".format(action))
            return

        self.gripper_pub.publish(cmd)
        self.gripper.go(wait=True)  # MoveIt visualisatie
        rospy.sleep(1)  # geef tijd om te bewegen

    def target_callback(self, msg):
        rospy.loginfo("Nieuwe target ontvangen.")

        # Zet de ijsco positie om naar het assenstel waaruit je plant
        source_frame = "camera_link"
        target_frame = self.arm.get_planning_frame()  # meestal 'world'

        self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

        # Maak een PointStamped in camera_link frame
        point_in_source = geometry_msgs.msg.PointStamped()
        point_in_source.header.frame_id = source_frame
        point_in_source.header.stamp = rospy.Time(0)
        point_in_source.point.x = msg.x / 1000.0
        point_in_source.point.y = msg.y / 1000.0
        point_in_source.point.z = -(msg.z - 250) / 1000.0  # Z negatief vanwege camera assenstelsel en -180mm vanwege lengte closed gripper (endeffector = tool0)

        # Transformeer positie naar world frame
        point_in_target = self.tf_listener.transformPoint(target_frame, point_in_source)

        # Oriëntatie berekenen
        yaw_rad = math.radians(msg.rz)
        q = tf.transformations.quaternion_from_euler(math.pi, 0, yaw_rad)

        # Stel doel-pose in
        pose = Pose()
        pose.position.x = point_in_target.point.x
        pose.position.y = point_in_target.point.y
        pose.position.z = point_in_target.point.z
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Beweeg arm naar ijsco
        self.arm.set_pose_target(pose)
        self.arm.go(wait=True)

        # Gripper sluiten
        self.move_gripper("close")

        # Terug naar home
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

        # Ga naar juiste bak
        self.go_to_bin(msg.bin_number)

        # Gripper openen
        self.move_gripper("open")

        # Terug naar home
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

        rospy.loginfo("Wacht op volgend ijsje...")

    def go_to_bin(self, number):
        target_name = "bin_{}".format(number)
        rospy.loginfo("Beweeg naar {}".format(target_name))
        self.arm.set_named_target(target_name)
        self.arm.go(wait=True)

if __name__ == "__main__":
    try:
        IceRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
