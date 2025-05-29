import time
import sys
import os

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

# Simplified import for RobotiqGripper - assumes it's in the current directory
def import_robotiq_gripper():
    """Import the RobotiqGripper class from the current directory."""
    
    # Get the current directory where this script is located
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Make sure the current directory is in the Python path
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    
    # Try to import the module
    try:
        from robotiq_gripper import RobotiqGripper
        print(f"Successfully imported RobotiqGripper from current directory")
        return RobotiqGripper
    except ImportError as e:
        print(f"ERROR: Could not import RobotiqGripper: {e}")
        print("Please make sure robotiq_gripper.py is in the same directory as this script.")
        sys.exit(1)

# Import the RobotiqGripper class
RobotiqGripper = import_robotiq_gripper()

ROBOT_IP = "192.168.56.101"  # Change this to your robot IP
GRIPPER_PORT = 63352

# Fixed parameters
HOME_POSITION = [-0.1540, -0.3633, 0.2363, 0.0060, -3.1392, 0.0656]  # From robot_control_home.py
DEFAULT_VELOCITY = 0.1
DEFAULT_ACCELERATION = 0.1

# Z-height parameters
TRAVEL_Z_HEIGHT = 0.2363  # Height for traveling between positions (from robot_control_home.py)
PICKUP_Z_HEIGHT = 0.1710  # Default height for pickup (from robot_control_main.py)
TARGET_Z_HEIGHT = 0.1910  # Default height for target placement (from robot_control_main.py)
MIN_SAFE_Z = 0.1660  # Absolute minimum safe Z height to avoid hitting the table
TABLE_Z = 0.1610  # Estimated table height (must stay above this)

class RobotController:
    def __init__(self, robot_ip=ROBOT_IP, gripper_port=GRIPPER_PORT):
        """Initialize robot controller with RTDE interfaces and gripper."""
        self.robot_ip = robot_ip
        self.gripper_port = gripper_port
        self.rtde_c = None
        self.rtde_r = None
        self.gripper = None
        
    def connect(self):
        """Connect to robot and gripper."""
        # Initialize robot RTDE interfaces
        try:
            self.rtde_c = RTDEControlInterface(self.robot_ip)
            self.rtde_r = RTDEReceiveInterface(self.robot_ip)
            print(f"Connected to robot at {self.robot_ip}")
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            return False
            
        # Connect to gripper
        self.gripper = RobotiqGripper(self.robot_ip, self.gripper_port)
        if not self.gripper.connect():
            print("Failed to connect to gripper.")
            self.disconnect()
            return False
            
        # Activate gripper if needed
        if self.gripper.get_status() != 3:
            print("Activating gripper...")
            if self.gripper.activate():
                print("Gripper activated.")
                time.sleep(2.0)
            else:
                print("Gripper activation failed.")
                self.disconnect()
                return False
                
        return True
    
    def disconnect(self):
        """Disconnect from robot and gripper."""
        if self.rtde_c:
            self.rtde_c.stopScript()
        if self.gripper:
            self.gripper.disconnect()
        print("Disconnected from robot and gripper.")
    
    def get_current_pose(self):
        """Get the current TCP pose of the robot."""
        if not self.rtde_r:
            print("Not connected to robot.")
            return None
        
        current_pose = self.rtde_r.getActualTCPPose()
        print("\nCurrent TCP Pose (x, y, z, rx, ry, rz):")
        print(["{:.4f}".format(p) for p in current_pose])
        return current_pose
    
    def validate_z_height(self, z, operation_name=""):
        """
        Check if Z height is safe.
        
        Args:
            z: Z height to validate (in meters)
            operation_name: Optional name of operation for better error messages
        
        Returns:
            bool: True if Z height is safe, False otherwise
        """
        if z < MIN_SAFE_Z:
            print(f"WARNING: Z height {z}m for {operation_name} is below minimum safe height {MIN_SAFE_Z}m.")
            print(f"This would cause the robot to hit the table at approximately {TABLE_Z}m.")
            return False
        return True
    
    def pickup(self, x, y, z=None, travel_z=None, rx=None, ry=None, rz=None):
        """
        Move to pickup position, close gripper, and move up.
        
        Args:
            x, y: Pickup position coordinates (in mm)
            z: Optional Z height for pickup (in m, will use PICKUP_Z_HEIGHT if not provided)
            travel_z: Optional Z height for travel (in m, will use TRAVEL_Z_HEIGHT if not provided)
            rx, ry, rz: Optional rotation values (will use current pose if not provided)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not all([self.rtde_c, self.rtde_r, self.gripper]):
            print("Not connected to robot or gripper.")
            return False
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Convert from mm to m for x, y
        x_m, y_m = x/1000, y/1000
        
        # Use provided z or default to PICKUP_Z_HEIGHT
        z_m = z if z is not None else PICKUP_Z_HEIGHT
        
        # Use provided travel_z or default to TRAVEL_Z_HEIGHT
        travel_z_m = travel_z if travel_z is not None else TRAVEL_Z_HEIGHT
        
        # Validate Z heights
        if not self.validate_z_height(z_m, "pickup"):
            return False
        
        # Use provided rotation or current rotation
        rx_val = rx if rx is not None else current_pose[3]
        ry_val = ry if ry is not None else current_pose[4]
        rz_val = rz if rz is not None else current_pose[5]
        
        # Move to position above pickup
        print(f"\nMoving above pickup position ({x}, {y})...")
        above_pickup = [x_m, y_m, travel_z_m, rx_val, ry_val, rz_val]
        self.rtde_c.moveL(above_pickup, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        # Move down to pickup position
        print(f"Moving down to pickup position (Z={z_m}m)...")
        pickup_pose = [x_m, y_m, z_m, rx_val, ry_val, rz_val]
        self.rtde_c.moveL(pickup_pose, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        # Close gripper
        print("Closing gripper...")
        self.gripper.close(speed=255, force=255)
        time.sleep(2)
        
        # Move back up
        print(f"Moving back up to travel height (Z={travel_z_m}m)...")
        self.rtde_c.moveL(above_pickup, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        return True
    
    def target(self, x, y, z=None, travel_z=None, rx=None, ry=None, rz=None):
        """
        Move to target position, open gripper, and move up.
        
        Args:
            x, y: Target position coordinates (in mm)
            z: Optional Z height for target (in m, will use TARGET_Z_HEIGHT if not provided)
            travel_z: Optional Z height for travel (in m, will use TRAVEL_Z_HEIGHT if not provided)
            rx, ry, rz: Optional rotation values (will use current pose if not provided)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not all([self.rtde_c, self.rtde_r, self.gripper]):
            print("Not connected to robot or gripper.")
            return False
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Convert from mm to m for x, y
        x_m, y_m = x/1000, y/1000
        
        # Use provided z or default to TARGET_Z_HEIGHT
        z_m = z if z is not None else TARGET_Z_HEIGHT
        
        # Use provided travel_z or default to TRAVEL_Z_HEIGHT
        travel_z_m = travel_z if travel_z is not None else TRAVEL_Z_HEIGHT
        
        # Validate Z heights
        if not self.validate_z_height(z_m, "target"):
            return False
        
        # Use provided rotation or current rotation
        rx_val = rx if rx is not None else current_pose[3]
        ry_val = ry if ry is not None else current_pose[4]
        rz_val = rz if rz is not None else current_pose[5]
        
        # Move to position above target
        print(f"\nMoving above target position ({x}, {y})...")
        above_target = [x_m, y_m, travel_z_m, rx_val, ry_val, rz_val]
        self.rtde_c.moveL(above_target, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        # Move down to target position
        print(f"Moving down to target position (Z={z_m}m)...")
        target_pose = [x_m, y_m, z_m, rx_val, ry_val, rz_val]
        self.rtde_c.moveL(target_pose, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        # Open gripper
        print("Opening gripper...")
        self.gripper.open(speed=255, force=255)
        time.sleep(2)
        
        # Move back up
        print(f"Moving back up to travel height (Z={travel_z_m}m)...")
        self.rtde_c.moveL(above_target, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        return True
    
    def rotate(self, rotation, axis='z'):
        """
        Rotate the robot at its current position.
        
        Args:
            rotation: Rotation value in radians
            axis: Rotation axis ('x', 'y', or 'z', default='z')
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not all([self.rtde_c, self.rtde_r]):
            print("Not connected to robot.")
            return False
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Create a new pose with the rotation updated
        new_pose = current_pose.copy()
        
        # Update the appropriate rotation value
        if axis.lower() == 'x':
            new_pose[3] = rotation
            print(f"Rotating around X axis to {rotation} radians...")
        elif axis.lower() == 'y':
            new_pose[4] = rotation
            print(f"Rotating around Y axis to {rotation} radians...")
        elif axis.lower() == 'z':
            new_pose[5] = rotation
            print(f"Rotating around Z axis to {rotation} radians...")
        else:
            print(f"Invalid rotation axis: {axis}. Must be 'x', 'y', or 'z'.")
            return False
        
        # Execute the rotation
        self.rtde_c.moveL(new_pose, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        
        return True
    
    def home(self):
        """
        Move the robot to the home position.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.rtde_c:
            print("Not connected to robot.")
            return False
        
        print("\nMoving to home position...")
        self.rtde_c.moveL(HOME_POSITION, DEFAULT_VELOCITY, DEFAULT_ACCELERATION)
        time.sleep(1)
        print("Home position reached.")
        
        return True
    
    def pick_and_place(self, pickup_x, pickup_y, target_x, target_y, 
                      pickup_z=None, target_z=None, travel_z=None,
                      rotation=None, axis='z'):
        """
        Perform a complete pick and place operation with optional rotation.
        
        Args:
            pickup_x, pickup_y: Pickup position coordinates (in mm)
            target_x, target_y: Target position coordinates (in mm)
            pickup_z: Optional Z height for pickup (in m)
            target_z: Optional Z height for target (in m)
            travel_z: Optional Z height for travel (in m)
            rotation: Optional rotation value in radians
            axis: Rotation axis ('x', 'y', or 'z', default='z')
        
        Returns:
            bool: True if successful, False otherwise
        """
        print("Starting pick and place operation...")
        
        # Step 1: Pickup
        if not self.pickup(pickup_x, pickup_y, pickup_z, travel_z):
            print("Pickup operation failed.")
            return False
        
        # Step 2: Rotate if requested
        if rotation is not None:
            if not self.rotate(rotation, axis):
                print("Rotation operation failed.")
                return False
        
        # Step 3: Place at target
        if not self.target(target_x, target_y, target_z, travel_z):
            print("Target operation failed.")
            return False
        
        # Step 4: Return to home
        if not self.home():
            print("Homing operation failed.")
            return False
        
        print("Pick and place operation completed successfully.")
        return True


def main():
    """Example usage of the RobotController class."""
    robot = RobotController()
    
    if not robot.connect():
        print("Failed to connect. Exiting.")
        return
    
    try:
        # Get current position
        current_pose = robot.get_current_pose()
        
        # Example pick and place operation
        print("\nEnter pickup position:")
        try:
            pickup_x = float(input("  X (mm): "))
            pickup_y = float(input("  Y (mm): "))
            custom_z = input("  Custom Z for pickup? (y/n): ").lower() == 'y'
            pickup_z = float(input("  Z (m): ")) if custom_z else None
        except ValueError:
            print("Invalid input.")
            return
        
        print("\nEnter target position:")
        try:
            target_x = float(input("  X (mm): "))
            target_y = float(input("  Y (mm): "))
            custom_z = input("  Custom Z for target? (y/n): ").lower() == 'y'
            target_z = float(input("  Z (m): ")) if custom_z else None
        except ValueError:
            print("Invalid input.")
            return
        
        # Ask for custom travel height
        custom_travel = input("\nCustom travel height? (y/n): ").lower() == 'y'
        travel_z = None
        if custom_travel:
            try:
                travel_z = float(input("  Travel Z (m): "))
            except ValueError:
                print("Invalid input. Using default travel height.")
                travel_z = None
        
        # Ask for rotation
        perform_rotation = input("\nPerform rotation? (y/n): ").lower() == 'y'
        rotation = None
        axis = 'z'
        if perform_rotation:
            try:
                rotation = float(input("  Rotation (radians): "))
                axis = input("  Axis (x/y/z, default=z): ") or 'z'
            except ValueError:
                print("Invalid input. Skipping rotation.")
                rotation = None
        
        # Execute the pick and place operation
        robot.pick_and_place(
            pickup_x, pickup_y, 
            target_x, target_y,
            pickup_z, target_z, travel_z,
            rotation, axis
        )
        
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main() 