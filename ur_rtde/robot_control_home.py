import time
import sys
import os

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

# Add your custom gripper module
GRIPPER_MODULE_PATH = os.path.expanduser("~/ros2_gripper_ws_main/robotiq_gripper.py")
sys.path.append(os.path.dirname(GRIPPER_MODULE_PATH))

try:
    from robotiq_gripper import RobotiqGripper
except ImportError:
    print("Error importing RobotiqGripper. Check your path.")
    sys.exit(1)

ROBOT_IP = "192.168.56.101"  # Change this to your robot IP
GRIPPER_PORT = 63352


def main():
    # Initialize robot RTDE interfaces
    rtde_c = RTDEControlInterface(ROBOT_IP)
    rtde_r = RTDEReceiveInterface(ROBOT_IP)

    # Connect to gripper
    gripper = RobotiqGripper(ROBOT_IP, GRIPPER_PORT)
    if not gripper.connect():
        print("Failed to connect to gripper.")
        sys.exit(1)

    # Activate gripper if needed
    if gripper.get_status() != 3:
        print("Activating gripper...")
        if gripper.activate():
            print("Gripper activated.")
            time.sleep(2.0)
        else:
            print("Gripper activation failed.")
            sys.exit(1)

    # Get current robot pose
    current_pose = rtde_r.getActualTCPPose()
    print("\nCurrent TCP Pose (x, y, z, rx, ry, rz) in m (note the teach pendant - tablet gives it in mm):")
    print(["{:.4f}".format(p) for p in current_pose])

    # Get new target pose from user
#    print("\nEnter new target pose:")
 #   try:
 #       x = float(input("  X: "))
 #       y = float(input("  Y: "))
 #       z = float(input("  Z: "))
 #       rx = float(input(" RX: "))
 #       ry = float(input(" RY: "))
 #       rz = float(input(" RZ: "))
 #   except ValueError:
 #       print("Invalid input.")
 #       rtde_c.stopScript()
 #       gripper.disconnect()
 #       return

    
    target_pose = [-0.1540, -0.3633, 0.2363, 0.0060, -3.1392, 0.0656]
    velocity = 0.1
    acceleration = 0.1

    print("\nMoving to target...")
    rtde_c.moveL(target_pose, velocity, acceleration)
    time.sleep(1)
    print("Reached target.")

    # Gripper control sequence
    print("\nClosing gripper...")
    #gripper.close(speed=255, force=255)
    time.sleep(2)

    print("Opening gripper...")
    gripper.open(speed=255, force=255)
    time.sleep(2)

    # Return to start
#    print("Returning to home position...")
#    rtde_c.moveL(current_pose, velocity, acceleration)

    rtde_c.stopScript()
    gripper.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()

