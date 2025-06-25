# Utility functions and constants for the logger module

STATE_NAME_MAP = {
    "Init": "Initialization",
    "MoveToWp": "Navigating to Waypoint",
    "SearchForObject": "Searching for Object",
    "CrossGate": "Crossing Gate",
    "SearchBuoyArea": "Searching Buoy Area",
    "InspectBuoy": "Inspecting Buoy",
    "InspectPipes": "Inspecting Pipes",
    "FollowMainPipe": "Following Main Pipe",
    "InterventionOnConsole": "Performing Intervention on Console",
    "UpdateLocalization": "Updating Localization",
    "Homing": "Returning to Home",
    "Halt": "Mission Halted"
}

TOPICS_NAMES = {
    "Pose": "/auv/global/pose_actual",
    "MissionStatus": "/auv/mission/status",
    "Obstacles": "/dtc/obstacles",
    "Camera": "/testing/sf/AUV/rgb_camera", #testing topic, to be changed to /camera/image_raw ?
    "Detections": "/detections",
}