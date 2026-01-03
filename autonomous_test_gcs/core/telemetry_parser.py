from typing import Dict

def parse_telemetry(telemetry: Dict[str, float]) -> str:
    ui_telemetry = (
                f"Lat: {telemetry['lat']:.6f} \t\t"
                f"Lon: {telemetry['lon']:.6f} \t\t"
                f"Alt: {telemetry['alt']:.2f} m\n"
                f"Vx: {telemetry['vx']:.2f} m/s \t\t"
                f"Vy: {telemetry['vy']:.2f} m/s \t\t"
                f"Vz: {telemetry['vz']:.2f} m/s\n"
                f"Roll: {telemetry['roll']:.2f}° \t\t"
                f"Pitch: {telemetry['pitch']:.2f}° \t\t"
                f"Yaw: {telemetry['yaw']:.2f}°\n"
                f"AccX: {telemetry['xacc']:.2f} m/s² \t\t"
                f"AccY: {telemetry['yacc']:.2f} m/s²\t\t"
            )
    return ui_telemetry