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
        KML_PATH = os.path.join(BASE_DIR, "data", "JU.kml")

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
            msg = self.the_connection.recv_match(blocking=False)
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

            log_telemetry(self.state)
            time.sleep(0.01)


    # -----------------------------------------------------------------------------
    # CONTROL THREAD
    # -----------------------------------------------------------------------------
    def _control_loop(self):        #Control thread to manage waypoints and modes
        HORIZ_TOL = 2.0   # meters
        VERT_TOL  = 0.5   # meters

        while self.running:

            # -------------------------
            # 1. Take ATOMIC SNAPSHOT
            # -------------------------
            with self.state_lock:
                t = self.state["telemetry"]
                cur_lat = t["lat"]
                cur_lon = t["lon"]
                cur_alt = t["alt"]

                tgt_lat = self.lat
                tgt_lon = self.lon
                tgt_alt = self.alt
                count   = self.target_count
                check = self.checked
                landing = self.landing

                targets = list(self.targets)   # shallow copy (safe)

            # --------------------------------------
            # 2. Compute distance OUTSIDE the lock
            # --------------------------------------
            d_h = haversine_dist(cur_lat, cur_lon, tgt_lat, tgt_lon)
            d_v = abs(cur_alt - tgt_alt)

            next_target = None
            need_mode_change = False

            # --------------------------------------
            # 3. Decide what to do (no lock needed)
            # --------------------------------------
            if not landing:         #Changes happening only if not landing only
                if count == -1 and cur_alt >= 3 - VERT_TOL:
                    count = 0
                    print("Alt reached")

                if count == 0:      #FOr initial waypoint
                
                    count = 1
                    if count < len(targets):
                        next_target = targets[count]
                        
                        self.send_coords(*next_target)

                elif d_h <= HORIZ_TOL and d_v <= VERT_TOL and count > 0 and not check:   #Reached a target
                    need_mode_change = True
                    check = True
                    count += 1
                    if count < len(targets):
                        next_target = targets[count]
                        
                elif (d_h > HORIZ_TOL or d_v > VERT_TOL) and count > 0:         #Enroute to next target
                    check = False

            # --------------------------------------
            # 4. Apply state changes ATOMICALLY
            # --------------------------------------
            with self.state_lock:
                self.target_count = count
                self.checked = check
                if next_target:
                    self.lat, self.lon, self.alt = next_target

            # --------------------------------------
            # 5. Only now run blocking operations
            # --------------------------------------
            if need_mode_change:
                
                time.sleep(5)
                if not self.landing:
                    self._set_mode("GUIDED")
                    if next_target:
                        # Important: call send_coords OUTSIDE the lock
                        self.send_coords(*next_target)

            time.sleep(0.2)

    #External Commands:
    def get_drone_state(self):
        # Return final dict including string ui_telemetry
        return {
            **self.state,
            "log": self.log
        }


    def start_drone(self):      #Arm and Takeoff and start mission
        with self.state_lock:
            
            self.state["status"] = "Active"
            self.set_mode("GUIDED")
            
            t = self.state["telemetry"]
            self.lat = t["lat"]
            self.lat0 = t["lat"]
            self.lon0 = t["lon"]
            self.lon = t["lon"]
            self.alt = t["alt"]
            self.landing = False        
            
            #The commented out code below is for setting multiple targets from the KML file in a spiral pattern (Plz verify once before running)
            
            #self.targets[0] = (self.lat,self.lon,self.alt)
            #targets = self.Planner.set_targets()
            #for target in targets:
            #   self.targets.append((target[0],target[1],10))
                       
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

        # Takeoff to 3m
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,0,0,0, 0,0, 3
        )
        ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if ack:
            if ack.result == 0:
                self.log += log_message("PixHawk", f"Drone Taking Off\n")
            else:
                self.log += log_message("PixHawk", f"Takeoff Failed\n")
        else:
            self.log += log_message("PixHawk", f"No Ack for Takeoff Command\n")

        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            20,         #Hz
            1
        )

        time.sleep(30)
        self.land_drone()


    def land_drone(self):           #Land the drone
        self.state["status"] = "Landing"
        self.set_mode("LAND")
        self.landing = True


    def return_to_launch(self, title="Sprayer Drone"):  #Return to Launch
        print(f"[{title}] RTL")
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

        self.the_connection.mav.set_mode_send(
            self.the_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode]
        )
        self.log += log_message("PixHawk", f"Mode Set to {mode}")
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