from controller import DroneController
from radio_comm import RadioComm
import time

class DroneManager:
    """
    Integrates DroneController with RadioComm
    Handles commands from base station
    """

    def __init__(self, radio_port="/dev/ttyAMA0"):
        self.controller = DroneController()

        # Import your existing RadioComm
        self.radio = RadioComm(port=radio_port)
        self.radio.start()

        self.command_handlers = {
            'START': self.handle_start,
            'LAND': self.handle_land,
            'RTL': self.handle_rtl,
            'PAUSE': self.handle_pause,
            'RESUME': self.handle_resume,
            'EMERGENCY': self.handle_emergency,
            'SET_WAYPOINTS': self.handle_set_waypoints,
            'SET_GEOFENCE': self.handle_set_geofence,
        }

    def handle_start(self, params):
        """Handle START command"""
        altitude = params.get('altitude', 5.0)
        self.controller.queue_command(
            self.controller.arm_and_takeoff, altitude)

    def handle_land(self, params):
        """Handle LAND command"""
        self.controller.queue_command(self.controller.land)

    def handle_rtl(self, params):
        """Handle RTL command"""
        self.controller.queue_command(self.controller.return_to_launch)

    def handle_pause(self, params):
        """Handle PAUSE command"""
        self.controller.pause_mission()

    def handle_resume(self, params):
        """Handle RESUME command"""
        self.controller.resume_mission()

    def handle_emergency(self, params):
        """Handle EMERGENCY command"""
        self.controller.emergency_stop()

    def handle_set_waypoints(self, params):
        """Handle SET_WAYPOINTS command"""
        waypoints = params.get('waypoints', [])
        self.controller.queue_command(self.controller.start_mission, waypoints)

    def handle_set_geofence(self, params):
        """Handle SET_GEOFENCE command"""
        radius = params.get('radius')
        min_alt = params.get('min_altitude')
        max_alt = params.get('max_altitude')
        self.controller.set_geofence(radius, min_alt, max_alt)

    def process_command(self, packet):
        """Process command from radio"""
        cmd = packet.get('command')
        params = packet.get('params', {})

        handler = self.command_handlers.get(cmd)
        if handler:
            try:
                handler(params)
            except Exception as e:
                print(f"Command handler error: {e}")
        else:
            print(f"Unknown command: {cmd}")

    def get_telemetry_packet(self) -> dict:
        """Get telemetry packet for radio transmission"""
        status = self.controller.get_status()

        return {
            'name': 'Sprayer',
            'password': 'vihang@2025',
            'state': status['state'],
            'battery': status['telemetry']['battery'],
            'telemetry': status['telemetry'],
            'mission': status['mission'],
            'healthy': status['healthy']
        }

    def run(self):
        """Main loop"""
        self.controller.connect()
        self.controller.start()

        try:
            while True:
                # Get commands from radio
                cmd = self.radio.get_latest_command()
                if cmd:
                    self.process_command(cmd)

                # Send telemetry
                packet = self.get_telemetry_packet()
                self.radio.send_packet(packet)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Shutting down...")

        finally:
            self.controller.stop()
