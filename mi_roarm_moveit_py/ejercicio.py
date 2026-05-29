#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import time
import math

class RoArmM2Sequencer(Node):
    def __init__(self):
        super().__init__('roarm_sequencer')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Conectando con MoveIt 2 (RoArm-M2-S)...")
        self._action_client.wait_for_server()

        # Grupo "hand" 
        self.joint_names = [
            "base_link_to_link1", 
            "link1_to_link2", 
            "link2_to_link3", 
            "link3_to_link4", 
            "link4_to_link5"
        ]
        
        # Grupo "gripper"
        self.gripper_joint_name = "link5_to_gripper_link"
  
        # --- POSICIONES CARTESIANAS ---
        self.P1_CART = [0.4, 0.0, 0.2]  
        self.PA_CART = [0.4, 0.0, 0.05]  
        self.P2_CART = [0.0, 0.4, 0.2]  
        self.PS_CART = [0.0, 0.4, 0.05]

        # --- ORIENTACIONES SEGÚN LA ROTACIÓN ---
        self.ORIENTACION_P1 = [0.0, 0.0, 0.0, 1.0]          
        self.ORIENTACION_P2 = [0.0, 0.0, 0.7071, 0.7071]    

        # Puntos Articulares (Radianes)
        self.HOME_JOINTS = [0.0, 0.0, 2.617, -1.047, 0.0]
        self.P1_JOINTS = [-0.0175, 0.4538, 1.8675, -0.7330, 0.0175]
        self.P2_JOINTS = [1.5533, 0.4538, 1.8675, -0.7330, 0.0175]

        # Configuración de la pinza 
        self.ABIERTA = 86.0
        self.CERRADA = 0.0  

    def mover_articular(self, positions, label):
        self.get_logger().info(f"Mov. Articular: {label}")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "hand" 
        
        constraints = Constraints()
        for i in range(len(positions)):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = float(positions[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        return self._send_wait(goal_msg)

    def mover_lineal(self, coords, orientacion, label):
       
        self.get_logger().info(f"Mov. Lineal: {label}")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "hand"
        
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = coords
        
    
        pose.pose.orientation.x = orientacion[0]
        pose.pose.orientation.y = orientacion[1]
        pose.pose.orientation.z = orientacion[2]
        pose.pose.orientation.w = orientacion[3]

        constraints = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = "hand_tcp"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01] 
        
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = "hand_tcp"
        ori_constraint.orientation = pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        return self._send_wait(goal_msg)

    def controlar_pinza(self, grados, label):
        self.get_logger().info(f"Pinza: {label} -> {grados}º")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper"
        
        valor_radianes = float(grados) * (math.pi / 180.0)
        
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.gripper_joint_name
        jc.position = valor_radianes
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        return self._send_wait(goal_msg)

    def _send_wait(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted: 
            self.get_logger().error(f"¡Meta Rechazada por MoveIt! Grupo: {goal_msg.request.group_name}")
            return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().status == 4

    def ejecutar_mision(self):
        self.get_logger().info("=== ARRANCANDO SEQUENCER INTEGRADO CON LA LÓGICA DEL PROFE ===")
        
        # Homing inicial
        self.mover_articular(self.HOME_JOINTS, "Homing del brazo")
        time.sleep(0.5)
       
        # Step 1: Ir de forma articular a la zona de recogida P1
        self.mover_articular(self.P1_JOINTS, "Movimiento articular a P1")
        time.sleep(0.5)
        
        # Step 2: Abrir la pinza
        self.controlar_pinza(self.ABIERTA, "Abriendo pinza")
        time.sleep(0.5)
        
        # Step 3: BAJAR RECTO EN LÍNEA TRASLACIONAL COMPLETA 
        self.mover_lineal(self.PA_CART, self.ORIENTACION_P1, "Bajando linealmente a PA")
        time.sleep(0.5)
        
        # Step 4: Cerrar pinza
        self.controlar_pinza(self.CERRADA, "Cerrando pinza")
        time.sleep(1.0)
        
        # Step 5: SUBIR RECTO EN LÍNEA TRASLACIONAL COMPLETA 
        self.mover_lineal(self.P1_CART, self.ORIENTACION_P1, "Subiendo linealmente a P1 con el objeto")
        time.sleep(0.5)

        # Step 6: Ir de forma articular a la zona de soltar P2 
        self.mover_articular(self.P2_JOINTS, "Movimiento articular a P2")
        time.sleep(0.5)

        # Step 7: BAJAR RECTO EN LÍNEA TRASLACIONAL COMPLETA
        self.mover_lineal(self.PS_CART, self.ORIENTACION_P2, "Bajando linealmente a PS")
        time.sleep(0.5)

        # Step 8: Abrir pinza
        self.controlar_pinza(self.ABIERTA, "Abriendo pinza")
        time.sleep(1.0)

        # Step 9: SUBIR RECTO EN LÍNEA TRASLACIONAL COMPLETA 
        self.mover_lineal(self.P2_CART, self.ORIENTACION_P2, "Subiendo linealmente a P2 sin el objeto")
        time.sleep(0.5)

        # Step 10: Homing final
        self.mover_articular(self.HOME_JOINTS, "Homing del brazo")
        self.controlar_pinza(self.CERRADA, "Cerrando pinza")
        
        time.sleep(0.5)

        self.get_logger().info("=== OPERACION TERMINADA ===")

def main():
    rclpy.init()
    node = RoArmM2Sequencer()
    node.ejecutar_mision()
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
