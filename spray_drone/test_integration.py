#!/usr/bin/env python3
"""
Integration test for DroneController with separated SprayController
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from controller import DroneController
from config import DroneConfig
from utils import Waypoint
import logging

def test_integration():
    """Test DroneController integration with SprayController"""
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create test config
    config = DroneConfig()
    config.spray_relay_pin = 6
    config.spray_duration = 5.0
    
    try:
        print("Creating DroneController...")
        controller = DroneController(config=config)
        
        # Test spray methods
        print("\n=== Testing Spray Methods ===")
        
        # Test spray status
        status = controller.get_spray_status()
        print(f"Spray Status: {status}")
        
        # Test is_spraying
        print(f"Is spraying: {controller.is_spraying()}")
        
        # Test manual spray
        print("Testing manual spray for 2 seconds...")
        controller.manual_spray(2.0)
        
        # Wait and check status
        import time
        time.sleep(1)
        print(f"During spray - Is spraying: {controller.is_spraying()}")
        
        # Wait for completion
        while controller.is_spraying():
            time.sleep(0.5)
        
        print("Manual spray completed")
        
        # Test stop spray
        print("Testing stop spray...")
        controller.manual_spray(10.0)  # Start long spray
        time.sleep(1)
        controller.stop_spray()  # Stop it
        print(f"After stop - Is spraying: {controller.is_spraying()}")
        
        # Test waypoint with spray
        print("\n=== Testing Waypoint Configuration ===")
        waypoint = Waypoint(
            lat=37.7749,
            lon=-122.4194,
            alt=50.0,
            spray_enabled=True,
            spray_duration=3.0
        )
        print(f"Test waypoint: {waypoint}")
        
        # Get final status
        print(f"\nFinal spray status: {controller.get_spray_status()}")
        
        # Cleanup
        controller.stop()
        print("Integration test completed successfully")
        
    except Exception as e:
        print(f"Integration test failed: {e}")
        raise

if __name__ == "__main__":
    test_integration()
