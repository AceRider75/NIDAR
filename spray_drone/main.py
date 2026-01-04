from config import DroneConfig
from controller import DroneController
import time
import threading

def example_basic_mission():
    """Example: Basic waypoint mission"""

    # Create controller with custom config
    config = DroneConfig(
        connection_string='127.0.0.1:14551',
        max_altitude=30.0,
        geofence_radius=200.0,
        waypoint_radius=3.0
    )

    controller = DroneController(config)

    try:
        # Connect
        if not controller.connect():
            print("Failed to connect")
            return

        # Start background threads
        controller.start()
        time.sleep(2)

        # Arm and takeoff
        if not controller.arm_and_takeoff(altitude=5.0):
            print("Failed to arm and takeoff")
            return

        # Define waypoints (lat, lon, alt)
        waypoints = [
            (22.497764, 88.372255, 5.0),
            (22.497701, 88.372401, 5.0),
            (22.497600, 88.372300, 5.0),
            (22.497943, 88.372148, 5.0),
        ]

        # Start mission
        if not controller.start_mission(waypoints):
            print("Failed to start mission")
            return

        # Monitor mission
        while controller.mission_active.is_set():
            status = controller.get_status()
            print(
                f"State: {status['state']}, Progress: {status['mission']['progress']}")
            time.sleep(2)

        # Land
        controller.land()

        # Wait for landing
        time.sleep(10)

    except KeyboardInterrupt:
        print("\nEmergency stop!")
        controller.emergency_stop()

    finally:
        controller.stop()


def example_with_monitoring():
    """Example: Mission with health monitoring"""

    controller = DroneController()

    try:
        controller.connect()
        controller.start()

        # Monitor thread
        def health_monitor():
            while controller.running.is_set():
                status = controller.get_status()

                # Check battery
                battery = status['telemetry']['battery']
                if battery < 20 and battery > 0:
                    print("WARNING: Low battery!")
                    controller.return_to_launch()

                # Check geofence
                if not status['healthy']:
                    print("WARNING: Connection unhealthy!")

                time.sleep(5)

        monitor = threading.Thread(target=health_monitor, daemon=True)
        monitor.start()

        # Execute mission
        controller.arm_and_takeoff(5.0)

        waypoints = [
            (22.497764, 88.372255, 5.0),
            (22.497701, 88.372401, 5.0),
        ]

        controller.start_mission(waypoints)

        # Wait for completion
        while controller.mission_active.is_set():
            time.sleep(1)

        controller.land()

    finally:
        controller.stop()

if __name__ == "__main__":
    print("Starting basic mission example...")
    example_basic_mission()

    # print("\nStarting mission with monitoring example...")
    # example_with_monitoring()