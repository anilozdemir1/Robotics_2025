#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from ijsjes_robot.msg import IceTarget  # custom message voor ijs-coördinaten
from ijsjes_robot.msg import HmiToRobot  # custom message voor hmi naar robot
from ijsjes_robot.msg import RobotToHmi  # custom message voor robot naar hmi
from geometry_msgs.msg import Pose
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
import tf
import geometry_msgs.msg
import math

class IceRobot:
    def __init__(self):
        # Initialiseer ROS node en MoveIt commander
        rospy.init_node('robot_control_node')
        moveit_commander.roscpp_initialize([])

        # Wacht tot move_group server beschikbaar is
        rospy.loginfo("Wachten op move_group server...")
        rospy.wait_for_service('/get_planning_scene')
        rospy.sleep(2)

        # Initialiseer MoveGroup commanders voor arm en gripper
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("endeffector")

        # Initialiseer TF listener voor coördinatentransformaties
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

        # Open gripper aan start
        self.move_gripper("open")

        # Breng arm naar home positie
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

        # HMI status flags, die bijgehouden worden vanuit de HMI callback (hmi to robot)
        self.emergency = False
        self.single_cycle = False
        self.start = False
        self.stop = False

        # Publisher en subscriber voor HMI communicatie
        self.hmi_to_robot_pub = rospy.Publisher('/hmi_to_robot', HmiToRobot, queue_size=10)
        rospy.Subscriber("/hmi_to_robot", HmiToRobot, self.hmi_callback)

        # Subscriber voor ijsje target data
        rospy.Subscriber("/ice_target", IceTarget, self.target_callback)

        # Publisher naar HMI met statusinformatie
        self.robot_to_hmi_pub = rospy.Publisher('/robot_to_hmi', RobotToHmi, queue_size=10)

        # Interne statusflags (robot to hmi)
        self.emergency_active = False
        self.stop_active = False
        self.running = False
        self.ready = False
        self.error = False
        self.current_action = "Wachten op input"

        # Timer om status periodiek te publiceren
        rospy.Timer(rospy.Duration(0.5), self.update_hmi_status)

        rospy.loginfo("Robot control node gestart en wacht op ijs-coördinaten en HMI-commando's.")

    def update_hmi_status(self, event):
        """
        Periodieke statusupdate naar HMI publisher
        """
        msg = RobotToHmi()
        msg.emergency_active = self.emergency
        msg.stop_active = self.stop
        msg.running = self.running
        msg.ready = not self.running and not self.emergency and not self.error
        msg.error = self.error
        msg.current_action = self.current_action
        self.robot_to_hmi_pub.publish(msg)

    def hmi_callback(self, msg):
        """
        Callback voor HMI commando's. 
        Houdt de huidige status bij en reageert direct op emergency door te stoppen.
        """
        self.emergency = msg.emergency
        self.single_cycle = msg.single_cycle
        self.start = msg.start
        self.stop = msg.stop

        if self.emergency:
            rospy.logwarn("EMERGENCY geactiveerd! Robot stopt direct.")
            self.arm.stop()
            self.gripper.stop()
            self.current_action = "EMERGENCY ACTIEF"
        elif self.stop:
            self.current_action = "Stoppen na cyclus"
        else:
            self.current_action = "Wachten op start/single_cycle"

    def target_callback(self, msg):
        """
        Callback voor ijsje target data. 
        Start sorteerproces zodra een nieuwe ijsco-positie binnenkomt via het topic /ice_target,
        Checkt eerst HMI flags en beslist of het sorteerproces uitgevoerd wordt.
        """
        # Bij emergency direct negeren
        if self.emergency:
            rospy.logwarn("Emergency actief, negeer target.")
            self.current_action = "Emergency actief - target genegeerd"
            return

        # Start sorteerproces als single_cycle of start actief is
        if self.single_cycle or self.start:
            self.current_action = "Start sorteerproces & ijsje gedetecteerd"
            self.running = True

            self.perform_sorting(msg)

            self.running = False

            # Reset single_cycle flag en stuur dit terug naar HMI
            if self.single_cycle:
                rospy.loginfo("Single cycle klaar, reset boolean.")
                self.single_cycle = False
                hmi_msg = HmiToRobot()
                hmi_msg.single_cycle = False
                self.hmi_to_robot_pub.publish(hmi_msg)

            # Reset stop flag en stuur dit terug naar HMI
            if self.stop:
                rospy.loginfo("Single cycle klaar, reset boolean.")
                self.stop = False
                hmi_msg = HmiToRobot()
                hmi_msg.stop = False
                self.hmi_to_robot_pub.publish(hmi_msg)

        else:
            self.current_action = "Wacht op start/single_cycle"

    def perform_sorting(self, msg):
        """
        Voert de hele sorteercyclus uit:
        - transformeert ijsco positie naar robot frame
        - plant en voert beweging uit naar ijsco (wacht als planning mislukt)
        - grijpt ijsco, terug naar home, beweegt naar juiste bak, opent gripper, terug naar home
        """
        source_frame = "camera_link"
        target_frame = self.arm.get_planning_frame()

        try:
            # Wacht op TF transform tussen frames
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

            # Maak PointStamped in camera frame
            point_in_source = geometry_msgs.msg.PointStamped()
            point_in_source.header.frame_id = source_frame
            point_in_source.header.stamp = rospy.Time(0)
            point_in_source.point.x = msg.x / 1000.0
            point_in_source.point.y = msg.y / 1000.0
            # Corrigeer z-waarde met offset gripper lengte en teken assenstelsel
            point_in_source.point.z = -(msg.z - 180) / 1000.0

            # Transformeer naar robot frame
            point_in_target = self.tf_listener.transformPoint(target_frame, point_in_source)

            # Bereken oriëntatie (quaternion) uit yaw hoek (radians)
            yaw_rad = math.radians(msg.rz)
            q = tf.transformations.quaternion_from_euler(math.pi, 0, yaw_rad)

            # Stel doelpose in
            pose = Pose()
            pose.position.x = point_in_target.point.x
            pose.position.y = point_in_target.point.y
            pose.position.z = point_in_target.point.z
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            # Probeer te plannen en uitvoeren, blijf proberen als plannen mislukt
            planned = False
            while not planned and not rospy.is_shutdown():
                self.current_action = "Plannen naar ijsje"
                self.arm.set_pose_target(pose)
                plan = self.arm.plan()

                if plan and len(plan.joint_trajectory.points) > 0:
                    planned = True  # planning is gelukt
                    self.current_action = "Planning gelukt, uitvoeren..."
                    self.error = False
                    success = self.arm.execute(plan, wait=True)

                    if not success:
                        self.error = True
                        self.current_action = "Uitvoering mislukt, probeer opnieuw..."
                        planned = False  # probeer opnieuw
                        rospy.sleep(1.0)
                else:
                    self.error = True
                    self.current_action = "Planning naar ijsco mislukt, probeer opnieuw..."
                    rospy.sleep(1.0)

            # Als planning geslaagd is, voer verdere stappen uit
            if planned:
                self.error = False
                self.current_action = "Grijp het ijsje"
                self.move_gripper("close")

                self.current_action = "Terug naar home"
                self.arm.set_named_target("home")
                self.arm.go(wait=True)

                self.current_action = "Beweeg naar bin {}".format(msg.bin_number)
                self.go_to_bin(msg.bin_number)

                self.current_action = "Laat ijsje los"
                self.move_gripper("open")

                self.current_action = "Terug naar home"
                self.arm.set_named_target("home")
                self.arm.go(wait=True)

                self.current_action = "Sorteren klaar, wacht op volgend ijsje..."

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.current_action = "TF transform error"
            rospy.logwarn("TF transform error: {}".format(e))

    def move_gripper(self, action):
        """
        Beweeg gripper naar open of gesloten positie via named targets
        """
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
        self.gripper.go(wait=True) # MoveIt visualisatie
        rospy.sleep(1)  # kort wachten om beweging te voltooien

    def go_to_bin(self, number):
        """
        Beweeg arm naar een vooraf gedefinieerde positie van de bak (bin)
        """
        target_name = "bin_{}".format(number)
        rospy.loginfo("Beweeg naar {}".format(target_name))
        self.arm.set_named_target(target_name)
        self.arm.go(wait=True)

        
if __name__ == "__main__":
    try:
        robot = IceRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass