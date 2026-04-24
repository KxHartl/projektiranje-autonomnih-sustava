#!/usr/bin/env python3
"""
Helper skripti za spremanje mapa iz SLAM Toolbox-a
Automatski kreira direktorij ako ne postoji
"""

import rclpy
from rclpy.client import Client
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
import os
import sys
from pathlib import Path


def save_map(map_path: str) -> bool:
    """
    Spremi mapu s automatskom kreacijom direktorija
    
    Args:
        map_path: Puna putanja do mape (bez .pgm/.yaml ekstenzije)
    
    Returns:
        True ako je uspješno, False ako nije
    """
    # Inicijalizacija ROS 2
    rclpy.init()
    node = rclpy.create_node('slam_map_saver')
    
    # Kreiraj direktorij ako ne postoji
    map_dir = os.path.dirname(map_path)
    Path(map_dir).mkdir(parents=True, exist_ok=True)
    node.get_logger().info(f'[SLAM_SAVER] Direktorij kreiiran/provjerio: {map_dir}')
    
    # Kreiraj klijent za save_map servis
    client = node.create_client(SaveMap, '/slam_toolbox/save_map')
    
    # Čekaj da je servis dostupan
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('[SLAM_SAVER] SLAM Toolbox servis nije dostupan!')
        rclpy.shutdown()
        return False
    
    # Kreiraj request
    request = SaveMap.Request()
    request.name = String(data=map_path)
    
    node.get_logger().info(f'[SLAM_SAVER] Pozivam save_map: {map_path}')
    
    # Pošalji request
    future = client.call_async(request)
    
    # Čekaj rezultat
    while rclpy.ok():
        if future.done():
            try:
                response = future.result()
                if response.result == 0:
                    node.get_logger().info('[SLAM_SAVER] ✓ Mapa uspješno spremljena!')
                    rclpy.shutdown()
                    return True
                else:
                    node.get_logger().error(
                        f'[SLAM_SAVER] ✗ save_map vratio kod: {response.result}'
                    )
                    rclpy.shutdown()
                    return False
            except Exception as e:
                node.get_logger().error(f'[SLAM_SAVER] Greška: {e}')
                rclpy.shutdown()
                return False
        rclpy.spin_once(node, timeout_sec=0.1)
    
    rclpy.shutdown()
    return False


def main():
    """
    CLI entry point
    
    Usage:
        slam_map_saver /path/to/map_05/map_05
    """
    if len(sys.argv) < 2:
        print('Usage: slam_map_saver <map_path>')
        print('Example: slam_map_saver /home/khartl/assignment_02_ws/.../mapped_maps/map_05/map_05')
        sys.exit(1)
    
    map_path = sys.argv[1]
    
    success = save_map(map_path)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
