import asyncio
from mavsdk import System
import KeyPressModule as kp

# Initialize global control variables
roll = pitch = yaw = 0
throttle = 0.5

# Initialize key press detection
kp.init()
drone = System()

async def handle_keyboard_input(drone):
    """Continuously read keyboard input to set drone control values."""
    global roll, pitch, throttle, yaw
    control_intensity = 0.5

    while True:
        roll = pitch = yaw = 0
        throttle = 0.5  # Neutral throttle value

        if kp.getKey("LEFT"):
            pitch = -control_intensity
        elif kp.getKey("RIGHT"):
            pitch = control_intensity

        if kp.getKey("UP"):
            roll = control_intensity
        elif kp.getKey("DOWN"):
            roll = -control_intensity

        if kp.getKey("w"):
            throttle = 1.0
        elif kp.getKey("s"):
            throttle = 0.0

        if kp.getKey("a"):
            yaw = -control_intensity
        elif kp.getKey("d"):
            yaw = control_intensity

        if kp.getKey("i"):
            asyncio.ensure_future(log_flight_mode(drone))

        if kp.getKey("r") and await drone.telemetry.landed_state():
            await drone.action.arm()

        if kp.getKey("l") and await drone.telemetry.in_air():
            await drone.action.land()

        await asyncio.sleep(0.1)

async def log_flight_mode(drone):
    """Print the current flight mode."""
    async for mode in drone.telemetry.flight_mode():
        print("Current Flight Mode:", mode)
        break  # Only print once

async def send_manual_control(drone):
    """Send the current control values to the drone continuously."""
    global roll, pitch, throttle, yaw
    while True:
        print(f"Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}")
        await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)

async def initialize_drone():
    """Connect to the drone and start control loops."""
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone successfully connected.")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate is sufficient for flight.")
            break

    asyncio.ensure_future(handle_keyboard_input(drone))
    asyncio.ensure_future(send_manual_control(drone))

async def main():
    await initialize_drone()

if __name__ == "__main__":
    asyncio.ensure_future(main())
    asyncio.get_event_loop().run_forever()
