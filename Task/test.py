import time
from pymavlink import mavutil

def connect(port=14550):
    print(f"[MAVLink] Connecting to udp:0.0.0.0:{port} ...")
    # Changed to 127.0.0.1 to ensure it binds correctly to the local MAVProxy router on Debian
    m = mavutil.mavlink_connection(f"udp:127.0.0.1:{port}")
    m.wait_heartbeat()
    print(f"[MAVLink] Connected to system {m.target_system}")
    return m

def set_mode(m, mode):
    mid = m.mode_mapping()[mode]
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mid)
    print(f"[MAVLink] Mode set to {mode}")

def arm(m):
    print("[MAVLink] Arming ...")
    
    # The second parameter is 0, which means it will perform all pre-arm safety checks
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    
    # Wait until armed flag is confirmed
    while True:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("[MAVLink] Armed ✅")
            break
        time.sleep(0.5)

def send_velocity(m, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    """
    Sends offboard velocity commands to ArduPilot.
    NED Frame: Positive X is Forward, Positive Y is Right, Positive Z is DOWN.
    """
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b010111000111, 0, 0, 0,
        vx, vy, vz, 0, 0, 0, 0, yaw_rate)

def takeoff(m, alt):
    print(f"[MAVLink] Taking off to {alt}m ...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)
    
    # Block until altitude is reached
    while True:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"  Alt: {current_alt:.2f} m")
            # Account for barometer/sensor drift, consider it reached at 95%
            if current_alt >= (alt - 0.2): 
                print(f"[MAVLink] Reached target altitude of {alt}m ✅")
                break

def slow_land(m):
    print("[MAVLink] Initiating slow GUIDED descent (0.08 m/s) ...")
    last_print = 0
    
    while True:
        # Reduced from 0.15 to 0.08 (8 cm/s) for a buttery smooth descent
        send_velocity(m, vz=0.08)
        
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            alt = msg.relative_alt / 1000.0
            now = time.time()
            if now - last_print > 0.5:
                print(f"  Descending... Alt: {alt:.2f} m")
                last_print = now
            
            # Lowered the handover from 0.25m to 0.15m to prevent the "LAND mode plunge"
            if alt < 0.15:
                break
        
        # Velocity commands must be sent continuously to prevent failsafes
        time.sleep(0.1)

    print("[MAVLink] Nearing ground. Handing over to auto-LAND to touchdown and disarm...")
    set_mode(m, "LAND")

def main():
    TARGET_ALTITUDE = 1.2
    HOLD_TIME = 10
    
    master = connect()
    
    # 1. Set to GUIDED mode for offboard MAVLink control
    set_mode(master, "GUIDED")
    time.sleep(1)
    
    # 2. Arm the motors
    arm(master)
    
    # 3. Execute takeoff
    takeoff(master, TARGET_ALTITUDE)
    
    # 4. Hold position
    print(f"[Test] Holding altitude at {TARGET_ALTITUDE}m for {HOLD_TIME} seconds...")
    time.sleep(HOLD_TIME)
    
    # 5. Execute slow landing
    slow_land(master)
    print("[Test] Landing sequence triggered. Script complete.")

if __name__ == "__main__":
    main()