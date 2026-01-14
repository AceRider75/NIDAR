"""
Spray Controller Module for Drone Spray System

This module provides a simple relay-based spray control system for precision agriculture drones.
It handles GPIO control, timing, and thread-safe operations for automated spraying.
"""

import threading
import time
import logging
from typing import Optional
from dataclasses import dataclass
from enum import Enum, auto

# Add GPIO import for relay control
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class SprayState(Enum):
    """Spray system states"""
    IDLE = auto()
    SPRAYING = auto()
    ERROR = auto()


@dataclass
class SprayConfig:
    """Configuration for spray system"""
    relay_pin: int = 18  # GPIO pin for relay control
    default_duration: float = 10.0  # Default spray duration in seconds
    max_duration: float = 60.0  # Maximum allowed spray duration
    min_duration: float = 0.5  # Minimum allowed spray duration


class SprayController:
    """
    Controls spray pump motor via Raspberry Pi GPIO relay
    
    Features:
    - Simple ON/OFF relay control
    - Thread-safe operation with proper locking
    - Configurable spray durations
    - Emergency stop capability
    - Usage statistics tracking
    - GPIO safety and cleanup
    """
    
    def __init__(self, config: SprayConfig = None, logger: logging.Logger = None):
        self.config = config or SprayConfig()
        self.logger = logger or logging.getLogger(__name__)
        
        # State management
        self.state = SprayState.IDLE
        self.state_lock = threading.Lock()
        
        # GPIO setup
        self.gpio_initialized = False
        
        # Threading
        self.spray_thread: Optional[threading.Thread] = None
        self.spray_active = threading.Event()
        self.stop_requested = threading.Event()
        
        # Statistics
        self.total_spray_time = 0.0
        self.spray_count = 0
        self.last_spray_start: Optional[float] = None
        self.last_spray_duration: Optional[float] = None
        
        # Initialize GPIO
        self._setup_gpio()
        
    def _setup_gpio(self):
        """Initialize GPIO pins for relay control"""
        if not GPIO_AVAILABLE:
            self.logger.warning("GPIO not available - spray system will run in simulation mode")
            self.gpio_initialized = False
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.config.relay_pin, GPIO.OUT)
            GPIO.output(self.config.relay_pin, GPIO.LOW)  # Ensure relay is OFF initially
            self.gpio_initialized = True
            self.logger.info(f"Spray relay initialized on GPIO pin {self.config.relay_pin}")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize spray GPIO: {e}")
            self.gpio_initialized = False
            raise
    
    def _change_state(self, new_state: SprayState):
        """Thread-safe state change with logging"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            
        if old_state != new_state:
            self.logger.debug(f"Spray state: {old_state.name} -> {new_state.name}")
    
    def get_state(self) -> SprayState:
        """Get current spray state"""
        with self.state_lock:
            return self.state
    
    def is_spraying(self) -> bool:
        """Check if currently spraying"""
        return self.spray_active.is_set()
    
    def _relay_on(self) -> bool:
        """Turn spray relay ON"""
        try:
            if self.gpio_initialized:
                GPIO.output(self.config.relay_pin, GPIO.HIGH)
                self.logger.info("Spray relay ON")
            else:
                self.logger.info("Spray ON (simulation mode - no GPIO)")
            return True
        except Exception as e:
            self.logger.error(f"Failed to turn spray relay ON: {e}")
            return False

    def _relay_off(self) -> bool:
        """Turn spray relay OFF"""
        try:
            if self.gpio_initialized:
                GPIO.output(self.config.relay_pin, GPIO.LOW)
                self.logger.info("Spray relay OFF")
            else:
                self.logger.info("Spray OFF (simulation mode - no GPIO)")
            return True
        except Exception as e:
            self.logger.error(f"Failed to turn spray relay OFF: {e}")
            return False
    
    def _validate_duration(self, duration: float) -> float:
        """Validate and clamp spray duration to safe limits"""
        if duration < self.config.min_duration:
            self.logger.warning(f"Duration {duration}s too short, using minimum {self.config.min_duration}s")
            return self.config.min_duration
        elif duration > self.config.max_duration:
            self.logger.warning(f"Duration {duration}s too long, using maximum {self.config.max_duration}s")
            return self.config.max_duration
        return duration
    
    def start_spray(self, duration: float = None) -> bool:
        """
        Start spraying for specified duration
        
        Args:
            duration: Spray duration in seconds (uses config default if None)
            
        Returns:
            True if spray started successfully, False otherwise
        """
        # Use default duration if not specified
        duration = duration or self.config.default_duration
        duration = self._validate_duration(duration)
        
        # Check if already spraying
        if self.is_spraying():
            self.logger.warning("Already spraying, ignoring start command")
            return False
        
        # Check for error state
        if self.get_state() == SprayState.ERROR:
            self.logger.error("Spray system in error state, cannot start")
            return False
        
        # Clear any previous stop request
        self.stop_requested.clear()
        
        # Start spray thread
        self.spray_thread = threading.Thread(
            target=self._spray_worker,
            args=(duration,),
            name="SprayWorker",
            daemon=True
        )
        
        try:
            self.spray_thread.start()
            self.logger.info(f"Started spraying for {duration}s")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start spray thread: {e}")
            self._change_state(SprayState.ERROR)
            return False
    
    def stop_spray(self) -> bool:
        """
        Stop spraying immediately
        
        Returns:
            True if stopped successfully
        """
        if not self.is_spraying():
            return True
            
        self.logger.info("Stopping spray...")
        self.stop_requested.set()
        
        # Emergency turn off relay
        self._relay_off()
        
        # Wait for spray thread to finish
        if self.spray_thread and self.spray_thread.is_alive():
            self.spray_thread.join(timeout=3.0)
            if self.spray_thread.is_alive():
                self.logger.warning("Spray thread did not stop gracefully")
        
        return not self.is_spraying()
    
    def emergency_stop(self):
        """Emergency stop - immediate shutdown"""
        self.logger.warning("EMERGENCY STOP - Spray system")
        self.stop_requested.set()
        self._relay_off()
        self.spray_active.clear()
        self._change_state(SprayState.IDLE)
    
    def _spray_worker(self, duration: float):
        """
        Worker thread for spray operation with timing and safety checks
        
        Args:
            duration: Spray duration in seconds
        """
        try:
            self._change_state(SprayState.SPRAYING)
            self.spray_active.set()
            self.last_spray_start = time.time()
            
            # Turn relay ON
            if not self._relay_on():
                self.logger.error("Failed to turn relay ON")
                return
                
            self.logger.info(f"Spraying for {duration} seconds...")
            
            # Wait for duration or stop request
            start_time = time.time()
            end_time = start_time + duration
            
            while time.time() < end_time and not self.stop_requested.is_set():
                time.sleep(0.1)  # Check stop request every 100ms
            
            # Turn relay OFF
            self._relay_off()
            
            # Update statistics
            actual_duration = time.time() - start_time
            self.total_spray_time += actual_duration
            self.spray_count += 1
            self.last_spray_duration = actual_duration
            
            if self.stop_requested.is_set():
                self.logger.info(f"Spray stopped early after {actual_duration:.1f}s")
            else:
                self.logger.info(f"Spray completed - Duration: {actual_duration:.1f}s")
            
        except Exception as e:
            self.logger.error(f"Error in spray worker: {e}")
            self._change_state(SprayState.ERROR)
            
        finally:
            # Ensure relay is OFF
            self._relay_off()
            self.spray_active.clear()
            self._change_state(SprayState.IDLE)
    
    def get_status(self) -> dict:
        """Get comprehensive spray system status"""
        return {
            'state': self.get_state().name,
            'is_spraying': self.is_spraying(),
            'gpio_available': GPIO_AVAILABLE,
            'gpio_initialized': self.gpio_initialized,
            'relay_pin': self.config.relay_pin,
            'default_duration': self.config.default_duration,
            'total_spray_time': self.total_spray_time,
            'spray_count': self.spray_count,
            'last_spray_start': self.last_spray_start,
            'last_spray_duration': self.last_spray_duration,
            'config': {
                'relay_pin': self.config.relay_pin,
                'default_duration': self.config.default_duration,
                'max_duration': self.config.max_duration,
                'min_duration': self.config.min_duration
            }
        }
    
    def reset_statistics(self):
        """Reset spray usage statistics"""
        self.total_spray_time = 0.0
        self.spray_count = 0
        self.last_spray_start = None
        self.last_spray_duration = None
        self.logger.info("Spray statistics reset")
    
    def test_spray(self, duration: float = 2.0) -> bool:
        """
        Test spray system with short duration
        
        Args:
            duration: Test duration in seconds (default 2s)
            
        Returns:
            True if test successful
        """
        self.logger.info(f"Testing spray system for {duration}s...")
        return self.start_spray(duration)
    
    def cleanup(self):
        """Clean up GPIO resources and stop any active spraying"""
        self.logger.info("Cleaning up spray controller...")
        
        # Stop any active spraying
        self.stop_spray()
        
        # Cleanup GPIO
        if self.gpio_initialized:
            try:
                GPIO.output(self.config.relay_pin, GPIO.LOW)  # Ensure relay is OFF
                GPIO.cleanup(self.config.relay_pin)
                self.logger.info("Spray GPIO cleanup completed")
            except Exception as e:
                self.logger.error(f"Error during GPIO cleanup: {e}")
        
        self.gpio_initialized = False
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.cleanup()
        except:
            pass  # Ignore errors in destructor
