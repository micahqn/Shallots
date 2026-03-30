"""
Robot configuration detection and hardware mapping.

This module detects which robot is running and provides the appropriate hardware configuration.
"""

import socket
import uuid
from enum import Enum, auto
from typing import Final

from pykit.logger import Logger


class Robot(Enum):
    """Enumeration of available robots."""
    LARRY = auto()  # Test robot
    COMP = auto()   # Competition robot
    UNKNOWN = auto()  # Fallback if detection fails

def has_subsystem(subsystem_name: str) -> bool:
    """
    Check if a subsystem is available on the current robot.

    :param subsystem_name: Name of the subsystem (e.g., "climber", "intake")
    :return: True if the subsystem exists on the current robot, False otherwise
    """

    # Add/remove subsystems as needed
    LARRY_SUBSYSTEMS = {
        "drivetrain",
        "vision",
    }

    COMP_SUBSYSTEMS = {
        "drivetrain",
        "vision",
        "intake",
        "pivot",
        "launcher",
    }
    

    if currentRobot == Robot.COMP:
        return subsystem_name.lower() in COMP_SUBSYSTEMS
    else:  # LARRY or UNKNOWN defaults to LARRY
        return subsystem_name.lower() in LARRY_SUBSYSTEMS

def get_mac_address():
    # Get the MAC address as a 48-bit integer
    mac_num = uuid.getnode()
    # Format the integer into a hexadecimal string with colons
    mac = ':'.join(f'{((mac_num >> i) & 0xff):02x}' for i in range(0, 12*4, 8))[::-1]
    return mac


def detect_robot() -> Robot:
    """
    Detect which robot we're running on.

    Detection methods (in order of preference):
    1. MAC address of RoboRIO (most reliable)
    2. Hostname (if set differently on each robot)
    3. Environment variable ROBOT_NAME

    :return: The detected robot
    """
    print("Attempting to detect which robot we are connected to")

    # Method 1: Check MAC address (RoboRIO MAC addresses are unique)
    try:
        mac_address = get_mac_address() or "Undefined"
        Logger.recordMetadata("MACAddress", mac_address)
        
        # You can find MAC addresses via: ssh admin@roborio-XXXX-frc.local "cat /sys/class/net/eth0/address"

        LARRY_MAC_ADDRESSES = [
            "00:08:f2:33:f9:d1",  # Replace with Larry's actual MAC
            # Add other possible MAC addresses for Larry if it has multiple interfaces
        ]
        COMP_MAC_ADDRESSES = [
            "00:08:f2:83:d8:07",  # Replace with Comp's actual MAC
            # Add other possible MAC addresses for Comp if it has multiple interfaces
        ]

        if mac_address in LARRY_MAC_ADDRESSES:
            print("Mac address is for Larry")
            return Robot.LARRY
        
        if mac_address in COMP_MAC_ADDRESSES:
            print("Mac address is for Dwayne.")
            return Robot.COMP
        
    except Exception:
        pass

    # Method 2: Check hostname
    try:
        hostname = socket.gethostname().lower()
        if "larry" in hostname:
            return Robot.LARRY
        if "comp" in hostname or "competition" in hostname:
            return Robot.COMP
        
    except Exception:
        pass

    # Method 3: Check environment variable (useful for testing)
    import os

    robot_name = os.environ.get("ROBOT_NAME", "").upper()
    if robot_name == "LARRY":
        return Robot.LARRY
    if robot_name == "COMP":
        return Robot.COMP

    # Fallback: Default to COMP for competition, or set to LARRY for testing
    return Robot.LARRY


# Detect robot at module load time
currentRobot: Final[Robot] = detect_robot()
