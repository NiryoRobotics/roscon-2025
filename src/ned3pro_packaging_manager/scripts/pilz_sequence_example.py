#!/usr/bin/env python3
"""
Exemple d'utilisation des séquences Pilz Industrial Motion Planner
avec le robot NED3 Pro pour le packaging.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped

# Import de notre classe personnalisée
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from packaging_node_full import PilzSequencePlanner


class PilzSequenceExample(Node):
    def __init__(self):
        super().__init__("pilz_sequence_example")
        self.logger = get_logger("pilz_example")
        
        # Initialiser le Pilz Sequence Planner
        try:
            self.pilz_planner = PilzSequencePlanner(self, "arm")
            self.logger.info("Pilz Sequence Planner initialisé avec succès")
        except Exception as e:
            self.logger.error(f"Erreur d'initialisation Pilz Sequence Planner: {e}")
            return

    def run_basic_example(self):
        """Exemple basique de séquence Pilz"""
        self.logger.info("=== Exemple basique de séquence Pilz ===")
        
        # Définir les poses de test
        grip_pose = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0]  # Position de préhension
        place_pose = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]   # Position de placement
        
        # Créer une séquence de packaging basique
        sequence = self.pilz_planner.create_packaging_sequence(grip_pose, place_pose)
        
        # Planifier la séquence
        success = self.pilz_planner.plan_sequence(sequence)
        
        if success:
            self.logger.info("✅ Séquence basique planifiée avec succès")
        else:
            self.logger.error("❌ Échec de la planification de séquence basique")

    def run_advanced_example(self):
        """Exemple avancé avec différents types de mouvements"""
        self.logger.info("=== Exemple avancé de séquence Pilz ===")
        
        # Définir les poses avancées
        advanced_poses = {
            'approach': [0.0, -0.3, 0.0, 0.0, 0.0, 0.0],  # Position d'approche
            'grip': [0.0, -0.5, 0.0, 0.0, 0.0, 0.0],      # Position de préhension
            'place': [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],       # Position de placement
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]        # Position de départ
        }
        
        # Créer une séquence avancée
        sequence = self.pilz_planner.create_advanced_packaging_sequence(advanced_poses)
        
        # Planifier la séquence
        success = self.pilz_planner.plan_sequence(sequence)
        
        if success:
            self.logger.info("✅ Séquence avancée planifiée avec succès")
        else:
            self.logger.error("❌ Échec de la planification de séquence avancée")

    def run_custom_sequence_example(self):
        """Exemple de séquence personnalisée avec différents types de mouvements"""
        self.logger.info("=== Exemple de séquence personnalisée ===")
        
        sequence = []
        
        # 1. Mouvement PTP vers position d'approche
        approach_item = self.pilz_planner.add_ptp_motion(
            joint_positions=[0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
            blend_radius=0.0,
            velocity_scaling=0.4,
            acceleration_scaling=0.4
        )
        sequence.append(approach_item)
        
        # 2. Mouvement LIN vers position de préhension (si disponible en pose cartésienne)
        grip_pose = PoseStamped()
        grip_pose.header.frame_id = "base_link"
        grip_pose.pose.position.x = 0.2
        grip_pose.pose.position.y = 0.0
        grip_pose.pose.position.z = 0.3
        grip_pose.pose.orientation.w = 1.0
        
        grip_lin_item = self.pilz_planner.add_lin_motion(
            pose=grip_pose,
            blend_radius=0.0,
            velocity_scaling=0.1,
            acceleration_scaling=0.1
        )
        sequence.append(grip_lin_item)
        
        # 3. Mouvement PTP vers position de placement
        place_item = self.pilz_planner.add_ptp_motion(
            joint_positions=[0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
            blend_radius=0.1,
            velocity_scaling=0.2,
            acceleration_scaling=0.2
        )
        sequence.append(place_item)
        
        # 4. Retour à la position de départ
        home_item = self.pilz_planner.add_ptp_motion(
            joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            blend_radius=0.0,
            velocity_scaling=0.3,
            acceleration_scaling=0.3
        )
        sequence.append(home_item)
        
        # Planifier la séquence personnalisée
        success = self.pilz_planner.plan_sequence(sequence)
        
        if success:
            self.logger.info("✅ Séquence personnalisée planifiée avec succès")
        else:
            self.logger.error("❌ Échec de la planification de séquence personnalisée")

    def run_circular_motion_example(self):
        """Exemple de mouvement circulaire"""
        self.logger.info("=== Exemple de mouvement circulaire ===")
        
        sequence = []
        
        # 1. Mouvement PTP vers position de départ
        start_item = self.pilz_planner.add_ptp_motion(
            joint_positions=[0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
            blend_radius=0.0,
            velocity_scaling=0.3,
            acceleration_scaling=0.3
        )
        sequence.append(start_item)
        
        # 2. Mouvement circulaire (si supporté)
        center_pose = PoseStamped()
        center_pose.header.frame_id = "base_link"
        center_pose.pose.position.x = 0.1
        center_pose.pose.position.y = 0.0
        center_pose.pose.position.z = 0.2
        center_pose.pose.orientation.w = 1.0
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.2
        target_pose.pose.orientation.w = 1.0
        
        circ_item = self.pilz_planner.add_circ_motion(
            center_pose=center_pose,
            target_pose=target_pose,
            blend_radius=0.0,
            velocity_scaling=0.2,
            acceleration_scaling=0.2
        )
        sequence.append(circ_item)
        
        # Planifier la séquence avec mouvement circulaire
        success = self.pilz_planner.plan_sequence(sequence)
        
        if success:
            self.logger.info("✅ Séquence avec mouvement circulaire planifiée avec succès")
        else:
            self.logger.error("❌ Échec de la planification de séquence circulaire")

    def run_all_examples(self):
        """Exécuter tous les exemples"""
        self.logger.info("🚀 Démarrage des exemples Pilz Sequence Planner")
        
        # Exemple basique
        self.run_basic_example()
        time.sleep(2.0)
        
        # Exemple avancé
        self.run_advanced_example()
        time.sleep(2.0)
        
        # Exemple personnalisé
        self.run_custom_sequence_example()
        time.sleep(2.0)
        
        # Exemple de mouvement circulaire
        self.run_circular_motion_example()
        
        self.logger.info("🏁 Tous les exemples terminés")


def main():
    rclpy.init()
    
    try:
        node = PilzSequenceExample()
        
        # Exécuter tous les exemples
        node.run_all_examples()
        
    except KeyboardInterrupt:
        node.logger.info("Exemples interrompus par l'utilisateur")
    except Exception as e:
        node.logger.error(f"Erreur dans les exemples: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
