#!/usr/bin/env python3
"""
Test script for the separated SprayController system
"""

from spray_controller import SprayController, SprayConfig
import logging
import time

def test_spray_controller():
    """Test the spray controller functionality"""
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)
    
    # Create spray controller with test config
    config = SprayConfig(
        relay_pin=18,
        default_duration=5.0,  # 5 second test
        max_duration=30.0,
        min_duration=1.0
    )
    
    try:
        logger.info("Creating SprayController...")
        spray = SprayController(config=config, logger=logger)
        
        # Test status
        logger.info("Initial status:")
        status = spray.get_status()
        for key, value in status.items():
            logger.info(f"  {key}: {value}")
        
        # Test spray functionality
        logger.info("Testing spray for 3 seconds...")
        if spray.start_spray(3.0):
            logger.info("Spray started successfully")
            
            # Monitor spray status
            while spray.is_spraying():
                logger.info(f"Spraying... State: {spray.get_state().name}")
                time.sleep(0.5)
            
            logger.info("Spray completed")
        else:
            logger.error("Failed to start spray")
        
        # Test manual stop
        logger.info("Testing manual stop...")
        spray.start_spray(10.0)  # Start long spray
        time.sleep(2)  # Wait 2 seconds
        spray.stop_spray()  # Stop manually
        logger.info("Manual stop test completed")
        
        # Test emergency stop
        logger.info("Testing emergency stop...")
        spray.start_spray(10.0)  # Start long spray
        time.sleep(1)  # Wait 1 second
        spray.emergency_stop()  # Emergency stop
        logger.info("Emergency stop test completed")
        
        # Final status
        logger.info("Final status:")
        final_status = spray.get_status()
        for key, value in final_status.items():
            logger.info(f"  {key}: {value}")
        
        # Cleanup
        spray.cleanup()
        logger.info("Test completed successfully")
        
    except Exception as e:
        logger.error(f"Test failed: {e}")
        raise

if __name__ == "__main__":
    test_spray_controller()
