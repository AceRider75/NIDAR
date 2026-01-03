import threading
import time
from pymavlink import mavutil
import math
import os
from utils import haversine_dist, log_telemetry, log_message
from mission_plan import MissionPlanner

class Controller:

    def __init__(self):
        # Make connection
        self.the_connection = mavutil.mavlink_connection('/dev/ttyACM0')        #Connect to PixHawk

        # Wait for first heartbeat
        self.the_connection.wait_heartbeat()
        print(f"Heartbeat from system {self.the_connection.target_system}, component {self.the_connection.target_component}")
        log_message("PixHawk",f"Heartbeat from system {self.the_connection.target_system}, component {self.the_connection.target_component}")
        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            20,         #Hz
            1
        )
        # Thread flags
        self.running = True

        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")

        self.Planner = MissionPlanner(KML_PATH)

        # Shared state
        self.state = { 
            "status": "Idle",
            "battery": -1,
            "telemetry": {
                "lat": 0.0,
                "lon": 0.0,
                "alt": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "xacc": 0.0,
                "yacc": 0.0,
            }
        }

        self.state_lock = threading.Lock()
        
        #Flag varaibles:
        self.checked = False        #To check if target reached
        self.landing = False        #To check if landing in progress
        
        self.log = ""

        self.lat0 = 0
        self.lon0 = 0
        self.lat = 0
        self.lon = 0
        self.alt = 0

        self.targets = [(1,1,1)]
        self.target_count = -1
        self.mission_active = False  # Control loop flag

        # Start telemetry thread
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        self.control_thread = threading.Thread(target = self._control_loop, daemon = True)
        self.control_thread.start()
        
    # ----------------------------------------------------------
    # TELEMETRY THREAD
    # ----------------------------------------------------------
    def _telemetry_loop(self):      #Telemetry thread to read data from PixHawk
        while self.running:
            msg = self.the_connection.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                data = msg.to_dict()

                # SYSTEM STATUS (Battery)
                if msg_type == "SYS_STATUS":
                    with self.state_lock:
                        self.state["battery"] = data.get("battery_remaining", -1)

                # GLOBAL POSITION (lat, lon, alt)
                elif msg_type == "GLOBAL_POSITION_INT":
                    with self.state_lock:
                        self.state["telemetry"]["lat"] = data["lat"] / 1e7
                        self.state["telemetry"]["lon"] = data["lon"] / 1e7
                        self.state["telemetry"]["alt"] = data["relative_alt"] / 1000.0  # mm → m
                        self.state["telemetry"]["vx"] = data["vx"] / 100.0   # cm/s → m/s
                        self.state["telemetry"]["vy"] = data["vy"] / 100.0
                        self.state["telemetry"]["vz"] = data["vz"] / 100.0

                # ATTITUDE (roll, pitch, yaw)
                elif msg_type == "ATTITUDE":
                    with self.state_lock:
                        self.state["telemetry"]["roll"] = math.degrees(data["roll"])
                        self.state["telemetry"]["pitch"] = math.degrees(data["pitch"])
                        self.state["telemetry"]["yaw"] = math.degrees(data["yaw"])

                # RAW IMU (accelerations)
                elif msg_type == "RAW_IMU":
                    with self.state_lock:
                        self.state["telemetry"]["xacc"] = data["xacc"] / 1000.0   # mg → m/s^2
                        self.state["telemetry"]["yacc"] = data["yacc"] / 1000.0
                
                # HEARTBEAT (Flight Mode)
                elif msg_type == "HEARTBEAT":
                    with self.state_lock:
                        # custom_mode is the flight mode bitmap
                        # base_mode tells us if it's armed etc
                        try:
                            mode_id = data["custom_mode"]
                            # We can get the string name if we want, but let's trust mavutil
                            # This might be slow if we do it every heartbeat, but it's safe
                            mode_string = self.the_connection.flightmode
                            self.state["flight_mode"] = mode_string
                        except:
                            pass

            log_telemetry(self.state)
            time.sleep(0.01)


    def _control_loop(self):
            # Larger tolerance = drone flies through waypoints without stopping
            HORIZ_TOL = 5.0   # meters - increased for smooth continuous flight
            VERT_TOL  = 1.0
            
            # Track the last target we successfully processed
            last_processed_count = -1 

            while self.running:
                if not self.mission_active:
                    time.sleep(0.1)
                    continue

                with self.state_lock:
                    t = self.state["telemetry"]
                    cur_lat, cur_lon, cur_alt = t["lat"], t["lon"], t["alt"]
                    
                    tgt_lat, tgt_lon, tgt_alt = self.lat, self.lon, self.alt
                    count = self.target_count
                    targets = list(self.targets)
                
                # Calculate distance to the CURRENT target
                d_h = haversine_dist(cur_lat, cur_lon, tgt_lat, tgt_lon)
                d_v = abs(cur_alt - tgt_alt)

                next_target = None
                need_mode_change = False

                # 1. Initial Start
                if count == -1 and cur_alt >= 3 - VERT_TOL:
                    count = 0
                    print("Alt reached, starting mission")

                # 2. First Waypoint Logic
                if count == 0:
                    count = 1
                    if count < len(targets):
                        next_target = targets[count]
                        self.send_coords(*next_target)

                # 3. Target Reached Logic (Multi-skip)
                # Check if we reached the current target (or any subsequent ones)
                # This allows skipping multiple close waypoints in one cycle
                while count < len(targets) and count > 0:
                    # Current target coordinates
                    t_lat, t_lon, t_alt = targets[count]
                    
                    # Calculate distance to THIS target
                    d_h = haversine_dist(cur_lat, cur_lon, t_lat, t_lon)
                    d_v = abs(cur_alt - t_alt)
                    
                    if d_h <= HORIZ_TOL and d_v <= VERT_TOL:
                        # We reached this target
                        if count > last_processed_count:
                            print(f"Reached Target {count}")
                            last_processed_count = count
                            need_mode_change = True
                        
                        # Advance to next target
                        count += 1
                    else:
                        # Haven't reached this one yet
                        break

                if count < len(targets):
                    next_target = targets[count]
                else:
                    print("All targets completed")

                # Update State
                with self.state_lock:
                    self.target_count = count
                    if next_target:
                        self.lat, self.lon, self.alt = next_target

                # Execute Command
                if need_mode_change:
                    # Removed the sleep(5) to fix the "stopping" feel, but you can keep it if you want pauses.
                    # If you keep sleep(5), the logic above still prevents the deadlock.
                    if next_target:
                        self.send_coords(*next_target)

                time.sleep(0.1)

    #External Commands:
    def get_drone_state(self):
        # Return final dict including string ui_telemetry
        return {
            **self.state,
            "log": self.log
        }
    
    def is_armable(self):
        # Fetch the heartbeat
        msg = self.the_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            # MAV_STATE_STANDBY (3) means the system is grounded and ready to arm.
            # system_status is a direct field in the HEARTBEAT message.
            self.log += log_message("PixHawk", "Fetched heartbeat for armable check")
            return msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY
        self.log += log_message("PixHawk", "Failed to fetch heartbeat for armable check")
        return False
    
    def take_off_with_retry(self, height=3, retries=3):
        # Takeoff to 3m
        if not height or height <= 0:
            self.log += log_message("PixHawk", f"Invalid takeoff height specified: {height}m\n")
            return None
        for i in range(retries):
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,0,0,float('nan'),
                float('nan'),float('nan'), height
            )
            ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=3)
            if ack:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.log += "Takeoff accepted!\n"
                    return ack
                elif ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    self.log += f"Drone busy or not ready (Attempt {i+1}).\n"
                else:
                    self.log += f"Critical failure. Result code: {ack.result}\n"
                    return None
            time.sleep(1) # Wait before retrying

        return None

            
    


    def start_drone(self):      #Arm and Takeoff and start mission
        with self.state_lock:
            
            self.state["status"] = "Active"
            self.mission_active = True  # Enable control loop
            self.set_mode("GUIDED")
            
            t = self.state["telemetry"]
            self.lat = t["lat"]
            self.lat0 = t["lat"]
            self.lon0 = t["lon"]
            self.lon = t["lon"]
            self.alt = t["alt"]
            self.landing = False        
            
            #The commented out code below is for setting multiple targets from the KML file in a spiral pattern (Plz verify once before running)
            
            self.targets[0] = (self.lat,self.lon,self.alt)
            
            # Pass start location for path optimization (start from closest polygon corner)
            # Smooth path with 8 corner points, 5m spacing
            
            targets = self.Planner.set_targets(
                spacing_meters=5.0,
                start_lat=self.lat0,
                start_lon=self.lon0
            )
            
            for target in targets:
                print("Target Added:", target)
                self.targets.append((target[0], target[1], 3))
            
            # Add return-to-home waypoint at the end
            self.targets.append((self.lat0, self.lon0, 3))
            print(f"Return-to-home waypoint added: ({self.lat0:.6f}, {self.lon0:.6f})")

            print(f"Total Targets Generated: {len(self.targets)}")

        #check if the drone is armable or not
        start_time = time.time()
        while not self.is_armable():
            self.log += log_message("PixHawk", "Waiting for drone to become armable...")
            if time.time() - start_time > 30:
                self.log += log_message("PixHawk", "Drone not armable after 30 seconds. Aborting takeoff.")
                return
            time.sleep(1)
            

        # Arm
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack:
            if ack.result == 0:
                self.log += log_message("PixHawk", f"Drone Armed\n")
            else:
                self.log += log_message("PixHawk", f"Arm Failed\n")
        else:
            self.log += log_message("PixHawk", f"No Ack for Arm Command\n")

        # # Takeoff to 3m
        # self.the_connection.mav.command_long_send(
        #     self.the_connection.target_system,
        #     self.the_connection.target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        #     0, 0,0,0,float('nan'), float('nan'),float('nan'), 3
        # )
        # ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        # if ack:
        #     if ack.result == 0:
        #         self.log += log_message("PixHawk", f"Drone Taking Off\n")
        #     else:
        #         self.log += log_message("PixHawk", f"Takeoff Failed\n")
        # else:
        #     self.log += log_message("PixHawk", f"No Ack for Takeoff Command\n")

        ack = self.take_off_with_retry(height=3, retries=3)

        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.log += log_message("PixHawk", "Takeoff Command Accepted. Climbing...\n")
            
            # 2. Monitor Altitude instead of sleeping blindly
            start_time = time.time()
            while time.time() - start_time < 30: # 30 sec safety timeout
                # Requesting GLOBAL_POSITION_INT for relative altitude
                msg = self.the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    # relative_alt is in millimeters
                    current_alt = msg.relative_alt / 1000.0 
                    if current_alt >= 2.85: # Target reached (with small buffer)
                        self.log += log_message("PixHawk", f"Reached target altitude: {current_alt}m\n")
                        break
                time.sleep(0.5)
        else:
            error_code = ack.result if ack else "TIMEOUT"
            self.log += log_message("PixHawk", f"Takeoff Failed or Rejected. Error Code: {error_code}\n")
            return # Stop execution if takeoff fails

        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            20,         #Hz
            1
        )

        #time.sleep(30)
        #self.land_drone()
        
        # Set mission speed (m/s) - increase for faster mission completion
    #     self.set_speed(8.0)  # 8 m/s ground speed


    # def set_speed(self, speed_m_s: float):
    #     """Set ground speed in m/s for waypoint navigation"""
    #     self.the_connection.mav.command_long_send(
    #         self.the_connection.target_system,
    #         self.the_connection.target_component,
    #         mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
    #         0,
    #         1,              # Speed type: 1 = ground speed
    #         speed_m_s,      # Speed in m/s
    #         -1,             # Throttle (-1 = no change)
    #         0, 0, 0, 0
    #     )
    #     self.log += log_message("PixHawk", f"Speed set to {speed_m_s} m/s")
    #     print(f"Ground speed set to {speed_m_s} m/s")


    def land_drone(self):           #Land the drone
        self.mission_active = False  # Disable control loop immediately
        self.state["status"] = "Landing"
        self.landing = True
        
        # Retry loop for landing
        max_retries = 3
        for i in range(max_retries):
            print(f"Attempting to LAND (Attempt {i+1}/{max_retries})...")
            self.set_mode("LAND")
            
            # Wait a bit to see if mode changes
            time.sleep(1)
            
            # Check if we are actually in LAND mode
            current_mode = "UNKNOWN"
            with self.state_lock:
                current_mode = self.state.get("flight_mode", "UNKNOWN")
            
            if current_mode == "LAND":
                print("LAND mode confirmed!")
                self.log += log_message("PixHawk", "LAND mode confirmed")
                return
            
            print(f"Mode is {current_mode}, retrying LAND...")
            time.sleep(1)
            
        print("CRITICAL: Failed to switch to LAND mode after retries!")
        self.log += log_message("PixHawk", "CRITICAL: Failed to switch to LAND mode")


    def return_to_launch(self, title="Sprayer Drone"):  #Return to Launch
        print(f"[{title}] RTL")
        self.mission_active = False  # Disable control loop
        self.state["status"] = "RTL"
        self.set_mode("RTL")
        self.landing = False


    def send_coords(self, lat, lon, alt=3, title="Sprayer Drone"):      #Send Waypoints to PixHawk
        print(f"[{title}] Sending waypoint → {lat}, {lon}, {alt}")
        self.the_connection.mav.set_position_target_global_int_send(
            0,
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0,0,0,
            0,0,0,
            0,0
        )
        with self.state_lock:
            self.lat = lat
            self.lon = lon
            self.alt = alt
            

    # ----------------------------------------------------------
    def set_mode(self, mode):               #Set Mode of PixHawk
        mapping = self.the_connection.mode_mapping()
        if mode not in mapping:
            self.log = log_message("PixHawk", f"Mode: {mode} Not Supported")
            return

        # Try to set mode
        self.the_connection.mav.set_mode_send(
            self.the_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode]
        )
        self.log += log_message("PixHawk", f"Sent Mode Set to {mode}")
        
        # We don't wait here because we verify in the calling function (like land_drone)
        # or we can verify here. Let's verify here for general robustness.
        
        # Wait for ACK or Mode Change
        # Note: set_mode_send doesn't always send an ACK in all firmware versions, 
        # but checking heartbeat is reliable.
        
        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            20,         #Hz
            1
        )


    def _wait_for_ack(self, command, timeout=3):        #Wait for Acknowledgement from PixHawk
        start = time.time()
        while time.time() - start < timeout:
            msg = self.the_connection.recv_match(
                type='COMMAND_ACK',
                blocking=False
            )
            if msg and msg.command == command:
                return msg
            time.sleep(0.01)
        return None


    def stop(self):             #Stop all threads
        self.running = False
        self.telemetry_thread.join(timeout=1)
        self.control_thread.join(timeout=1)