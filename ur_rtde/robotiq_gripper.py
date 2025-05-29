#!/usr/bin/env python3

import socket
import time

class RobotiqGripper:
    """
    Class to control a Robotiq gripper using the TCP interface.
    """
    
    def __init__(self, robot_ip, port=63352):
        """
        Initialize the gripper connection.
        
        Args:
            robot_ip (str): IP address of the robot controller
            port (int): TCP port for the Robotiq gripper (default: 63352)
        """
        self.robot_ip = robot_ip
        self.port = port
        self.socket = None
    
    def connect(self):
        """
        Connect to the gripper.
        
        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robot_ip, self.port))
            print(f"Connected to gripper at {self.robot_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to gripper: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the gripper."""
        if self.socket:
            try:
                # Try to reset the gripper before disconnecting
                self.send_command("SET ACT 0")
            except:
                pass
            self.socket.close()
            self.socket = None
            print("Disconnected from gripper")
    
    def send_command(self, command, wait_time=0.1):
        """
        Send a command to the gripper and return the response.
        
        Args:
            command (str): Command to send
            wait_time (float): Time to wait after sending command
            
        Returns:
            str: Response from the gripper
        """
        if not self.socket:
            print("Not connected to gripper")
            return None
        
        try:
            self.socket.send((command + "\r\n").encode())
            time.sleep(wait_time)
            response = self.socket.recv(1024).decode().strip()
            return response
        except Exception as e:
            print(f"Command failed: {e}")
            return None
    
    def activate(self):
        """
        Complete gripper activation sequence.
        
        Returns:
            bool: True if activation was successful, False otherwise
        """
        # Reset the gripper
        response = self.send_command("SET ACT 0")
        if response != "ack":
            print(f"Reset failed: {response}")
            return False
        time.sleep(1.0)
        
        # Activate the gripper
        response = self.send_command("SET ACT 1")
        if response != "ack":
            print(f"Activation failed: {response}")
            return False
        time.sleep(2.0)
        
        # Check if the gripper is activated
        status = self.get_status()
        if status != 3:
            print(f"Gripper not ready. Status: {status}")
            return False
        
        # Set GTO mode
        response = self.send_command("SET GTO 1")
        if response != "ack":
            print(f"Setting GTO mode failed: {response}")
            return False
        
        print("Gripper activated successfully!")
        return True
    
    def reset(self):
        """
        Reset the gripper.
        
        Returns:
            bool: True if reset was successful, False otherwise
        """
        response = self.send_command("SET ACT 0")
        return response == "ack"
    
    def get_status(self):
        """
        Get the gripper status.
        
        Returns:
            int: Status code (0-3) or None if command failed
        """
        response = self.send_command("GET STA")
        if response and response.startswith("STA"):
            try:
                return int(response.split()[1])
            except:
                pass
        return None
    
    def get_position(self):
        """
        Get the current position of the gripper.
        
        Returns:
            int: Position (0-255) or None if command failed
        """
        response = self.send_command("GET POS")
        if response and response.startswith("POS"):
            try:
                return int(response.split()[1])
            except:
                pass
        return None
    
    def get_object_status(self):
        """
        Get the object detection status.
        
        Returns:
            int: Object status code (0-3) or None if command failed
            0: No object detected
            1: Object detected while opening
            2: Object detected while closing
            3: Object detected while stopped
        """
        response = self.send_command("GET OBJ")
        if response and response.startswith("OBJ"):
            try:
                return int(response.split()[1])
            except:
                pass
        return None
    
    def move(self, position, speed=255, force=255, wait=True, timeout=5.0):
        """
        Move the gripper to a specific position.
        
        Args:
            position (int): Position to move to (0-255, 0=open, 255=closed)
            speed (int): Speed of the move (0-255)
            force (int): Force to apply (0-255)
            wait (bool): Whether to wait for the move to complete
            timeout (float): Maximum time to wait for the move to complete
            
        Returns:
            bool: True if move was successful, False otherwise
        """
        # Ensure the gripper is activated and in GTO mode
        status = self.get_status()
        if status != 3:
            print(f"Gripper not ready. Status: {status}")
            if not self.activate():
                return False
        
        # Validate and constrain parameters
        position = max(0, min(255, int(position)))
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))
        
        # Send the move command
        command = f"SET POS {position} {speed} {force}"
        response = self.send_command(command)
        if response != "ack":
            print(f"Move command failed: {response}")
            return False
        
        # Wait for the move to complete
        if wait:
            start_time = time.time()
            while time.time() - start_time < timeout:
                time.sleep(0.1)
                current_pos = self.get_position()
                if current_pos is None:
                    continue
                
                # Check if we're at or near the target position
                if abs(current_pos - position) <= 5:
                    return True
                
                # Check if we have an object in the way
                obj_status = self.get_object_status()
                if obj_status in [1, 2, 3]:
                    print(f"Object detected during move (status {obj_status})")
                    return True
            
            print(f"Move timed out after {timeout} seconds")
        
        return True
    
    def open(self, speed=255, force=255, wait=True):
        """
        Open the gripper fully.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.move(0, speed, force, wait)
    
    def close(self, speed=255, force=255, wait=True):
        """
        Close the gripper fully.
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.move(255, speed, force, wait)


if __name__ == "__main__":
    # Example usage
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 robotiq_gripper.py ROBOT_IP")
        sys.exit(1)
    
    robot_ip = sys.argv[1]
    
    gripper = RobotiqGripper(robot_ip)
    if not gripper.connect():
        sys.exit(1)
    
    try:
        print("Activating gripper...")
        if not gripper.activate():
            print("Failed to activate gripper")
            sys.exit(1)
        
        print(f"Initial position: {gripper.get_position()}")
        
        print("Opening gripper...")
        gripper.open()
        print(f"Position after opening: {gripper.get_position()}")
        
        print("Closing gripper...")
        gripper.close()
        print(f"Position after closing: {gripper.get_position()}")
        
        print("Opening halfway...")
        gripper.move(128)
        print(f"Position after half-open: {gripper.get_position()}")
        
        print("Demo completed")
    
    finally:
        gripper.disconnect() 