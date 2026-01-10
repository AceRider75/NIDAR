# main.py - UPDATED VERSION

import time
from controller import DroneController, DroneConfig
import os

def main():
    print("Starting Controller...")

    # Configure
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")
    
    config = DroneConfig(
        connection_string='/dev/ttyACM0',
        geofence_mode="polygon",
        kml_file=KML_PATH,
        polygon_name="Field",
        max_altitude=30.0,
        waypoint_radius=3.0,
        default_altitude=3.0,
        optimize_waypoint_order=True
    )
    
    controller = DroneController(config)

    try:
        # Connect
        if not controller.connect():
            print("Failed to connect")
            return
        
        # Start threads
        controller.start()
        time.sleep(2)
        
        # Wait for Home Position
        # if not controller.request_home_position(timeout=60):
        #     print("Failed to get Home Position")
        #     return
            
        # Hard check for valid home position
        # telemetry = controller.get_telemetry()
        # if abs(telemetry['home_lat']) < 0.001 and abs(telemetry['home_lon']) < 0.001:
        #     print("ERROR: Home position is still (0,0) after request")
        #     return

        # Define mission waypoints
        waypoints = [
            # (22.497764, 88.372255, 3.0),
            # (22.497701, 88.372401, 3.0),
            # (22.497600, 88.372300, 3.0),
            # (22.497943, 88.372148, 3.0),
            # # (22.597600, 88.372300, 3.0),#outside geofence
            # # (22.497943, 88.472148, 3.0),#outside geofence
            # (22.498022, 88.372386, 3.0),
        ]
        
        # Load mission (validates against KML polygon)
        if not controller.load_mission_from_points(waypoints, validate_geofence=True):
            print("Failed to load mission")
            return
        
        # Add return home
        # controller.add_return_home_waypoint()
        
        # Arm and takeoff
        print("Arming and taking off...")
        if not controller.arm_and_takeoff(altitude=.0):
            print("Failed to arm and takeoff")
            return
        
        # Start mission
        print("Starting mission...")
        if not controller.start_mission():
            print("Failed to start mission")
            return
        
        # Monitor mission
        while controller.mission_active.is_set():
            status = controller.get_status()
            
            print(
                f"State: {status['state']} | "
                f"Progress: {status['mission']['progress']} | "
                f"Pos: ({status['telemetry']['lat']:.6f}, {status['telemetry']['lon']:.6f}, {status['telemetry']['alt']:.2f}m) | "
                f"Battery: {status['telemetry']['battery']}% | "
                f"Healthy: {status['healthy']}"
            )
            
            time.sleep(2)
        
        print("Mission complete!")
        
        # return to home and land
        print("Returning to home and landing...")
        if not controller.return_to_home_and_land():
            print("Failed to return home and land")
            controller.emergency_stop()
            return
        
        # Wait for landing
        time.sleep(10)

    except KeyboardInterrupt:
        print("Interrupted by user")
        print("Emergency landing...")
        controller.emergency_stop()
        time.sleep(10)

    finally:
        print("Stopping controller...")
        controller.stop()
        print("All good. Bye bye!")

if __name__ == "__main__":
    main()