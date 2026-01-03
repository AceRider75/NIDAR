# main.py
import time
from controller import Controller
# from src.image_processing import spot_tracker

def main():
    print("Starting Controller in SITL mode...")

    drone = Controller()

    try:
        # Give SITL some time to stabilize
        print("Waiting for telemetry...")
        time.sleep(5)

        # Print initial state
        state = drone.get_drone_state()
        print("Initial State:", state)

        # Start mission (ARM + TAKEOFF)
        print("Starting drone...")
        drone.start_drone()

        # Monitor telemetry until mission completes
        # Safety timeout: 30 minutes max
        MAX_MISSION_TIME = 30 * 60  # 30 minutes in seconds
        start = time.time()
        
        print("Mission in progress... Monitoring telemetry")
        
        while time.time() - start < MAX_MISSION_TIME:
            state = drone.get_drone_state()

            telem = state["telemetry"]
            
            # Get mission progress from controller
            with drone.state_lock:
                current_target = drone.target_count
                total_targets = len(drone.targets)
            
            print(
                f"ALT: {telem['alt']:.2f} m | "
                f"LAT: {telem['lat']:.6f} | "
                f"LON: {telem['lon']:.6f} | "
                f"MODE: {state['status']} ({state.get('flight_mode', 'UNK')}) | "
                f"BAT: {state['battery']}% | "
                f"TARGET: {current_target}/{total_targets}"
            )
            
            # Check if mission is complete
            if current_target >= total_targets and current_target > 0:
                print("Mission complete! All targets reached.")
                break

            time.sleep(1)
        
        # # Safety check: did we timeout?
        # if time.time() - start >= MAX_MISSION_TIME:
        #     print("Safety timeout reached (30 minutes). Landing...")

        # # Land after mission completion
        # print("Landing drone...")
        # drone.land_drone()
        
        # # Wait for landing to complete
        # print("Waiting for landing to complete...")
        # while True:
        #     state = drone.get_drone_state()
        #     alt = state["telemetry"]["alt"]
        #     print(f"Landing... ALT: {alt:.2f} m")
        #     if alt < 0.3:  # Consider landed when below 0.3m
        #         print("Drone has landed.")
        #         break
        #     time.sleep(1)

    except KeyboardInterrupt:
        print("Interrupted by user")
        print("Emergency landing...")
        drone.land_drone()
        time.sleep(10)

    finally:
        print("Stopping controller...")
        drone.stop()
        print("Test finished.")

if __name__ == "__main__":
    main()
    # spot_tracker.main()
