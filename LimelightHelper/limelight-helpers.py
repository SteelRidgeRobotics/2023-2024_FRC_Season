
from ntcore import NetworkTable, NetworkTableInstance, NetworkTableEntry
from wpimath.geometry import Translation2d, Translation3d, Pose2d, Pose3d, Rotation2d, Rotation3d
from wpinet import PortForwarder
from typing import List
import math
from wpilib import *
import time
import json
import wpiutil
import socket
from errno import *
from fcntl import *
from struct import *



def sanitize_name(name: str) -> str:
    if name == "":
        return "limelight"
    return name

def to_pose_3d(in_data: List[float]) -> Pose3d:
    if len(in_data) < 6:
        return Pose3d()
    return Pose3d(
        Translation3d(in_data[0], in_data[1], in_data[2]),
        Rotation3d(math.radians(in_data[3]), math.radians(in_data[4]), math.radians(in_data[5]))
    )

def to_pose_2d(in_data: List[float]) -> Pose2d:
    if len(in_data) < 6:
        return Pose2d()
    return Pose2d(
        Translation2d(in_data[0], in_data[1]),
        Rotation2d(math.radians(in_data[5]))
    )

def get_limelight_nt_table(table_name: str) -> NetworkTable:
    return NetworkTableInstance.getDefault().getTable(sanitize_name(table_name))

def get_limelight_nt_table_entry(table_name: str, entry_name: str) -> NetworkTableEntry:
    return get_limelight_nt_table(table_name).getEntry(entry_name)

def get_limelight_nt_double(table_name: str, entry_name: str):
    return get_limelight_nt_table_entry(table_name, entry_name).getDouble(0.0)

def get_limelight_nt_double_array(table_name: str, entry_name: str):
    return get_limelight_nt_table_entry(table_name, entry_name).getDoubleArray([])

def get_limelight_nt_string(table_name: str, entry_name: str):
    return get_limelight_nt_table_entry(table_name, entry_name).getString("")

def set_limelight_nt_double(table_name: str, entry_name: str, val: float):
    get_limelight_nt_table_entry(table_name, entry_name).setDouble(val)

def set_limelight_nt_double_array(table_name: str, entry_name: str, vals: List[float]):
    get_limelight_nt_table_entry(table_name, entry_name).setDoubleArray(vals)

def get_tx(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "tx")

def get_tv(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "tv")

def get_ty(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "ty")

def get_ta(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "ta")

def get_latency_pipeline(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "tl")

def get_latency_capture(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "cl")

def get_json_dump(limelight_name: str = "") -> str:
    return get_limelight_nt_string(limelight_name, "json")

def get_bot_pose(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "botpose")

def get_bot_pose_wpi_red(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "botpose_wpired")

def get_bot_pose_wpi_blue(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "botpose_wpiblue")

def get_bot_pose_target_space(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "botpose_targetspace")

def get_camera_pose_target_space(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "camerapose_targetspace")

def get_camera_pose_robot_space(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "camerapose_robotspace")

def get_target_pose_camera_space(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "targetpose_cameraspace")

def get_target_pose_robot_space(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "targetpose_robotspace")

def get_target_color(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "tc")

def get_fiducial_id(limelight_name: str = "") -> float:
    return get_limelight_nt_double(limelight_name, "tid")

def get_neural_class_id(limelight_name: str = "") -> str:
    return get_limelight_nt_string(limelight_name, "tclass")

def set_pipeline_index(limelight_name: str, index: int):
    set_limelight_nt_double(limelight_name, "pipeline", index)

def set_priority_tag_id(limelight_name: str, ID: int):
    set_limelight_nt_double(limelight_name, "priorityid", ID)

def set_led_mode_pipeline_control(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "ledMode", 0)

def set_led_mode_force_off(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "ledMode", 1)

def set_led_mode_force_blink(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "ledMode", 2)

def set_led_mode_force_on(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "ledMode", 3)

def set_stream_mode_standard(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "stream", 0)

def set_stream_mode_pip_main(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "stream", 1)

def set_stream_mode_pip_secondary(limelight_name: str = ""):
    set_limelight_nt_double(limelight_name, "stream", 2)

def set_crop_window(limelight_name: str, crop_x_min: float, crop_x_max: float, crop_y_min: float, crop_y_max: float):
    crop_window = [crop_x_min, crop_x_max, crop_y_min, crop_y_max]
    set_limelight_nt_double_array(limelight_name, "crop", crop_window)

def set_camera_pose_robot_space(limelight_name: str, forward: float, side: float, up: float, roll: float, pitch: float, yaw: float):
    entries = [forward, side, up, roll, pitch, yaw]
    set_limelight_nt_double_array(limelight_name, "camerapose_robotspace_set", entries)

def set_python_script_data(limelight_name: str, outgoing_python_data: List[float]):
    set_limelight_nt_double_array(limelight_name, "llrobot", outgoing_python_data)

def get_python_script_data(limelight_name: str = "") -> List[float]:
    return get_limelight_nt_double_array(limelight_name, "llpython")



def extract_bot_pose_entry(in_data: List[float], position: int) -> float:
    if len(in_data) < position + 1:
        return 0.0
    return in_data[position]



class PoseEstimate:
    def __init__(self, pose: Pose2d, timestamp_seconds: float = 0.0, latency: float = 0.0, tag_count: int = 0, tag_span: float = 0.0, avg_tag_dist: float = 0.0, avg_tag_area: float = 0.0):
        self.pose = pose
        self.timestamp_seconds = timestamp_seconds
        self.latency = latency
        self.tag_count = tag_count
        self.tag_span = tag_span
        self.avg_tag_dist = avg_tag_dist
        self.avg_tag_area = avg_tag_area

def get_bot_pose_estimate(limelight_name: str, entry_name: str) -> PoseEstimate:
    pose_entry = get_limelight_nt_table_entry(limelight_name, entry_name)
    pose_array = get_limelight_nt_double_array(limelight_name, entry_name)
    pose = to_pose_2d(pose_array)

    latency = extract_bot_pose_entry(pose_array, 6)
    tag_count = int(extract_bot_pose_entry(pose_array, 7))
    tag_span = extract_bot_pose_entry(pose_array, 8)
    tag_dist = extract_bot_pose_entry(pose_array, 9)
    tag_area = extract_bot_pose_entry(pose_array, 10)

    timestamp = ((pose_entry.getLastChange() / 1000000.0) - (latency / 1000.0))

    return PoseEstimate(pose, timestamp, latency, tag_count, tag_span, tag_dist, tag_area)

def get_bot_pose_estimate_wpi_blue(limelight_name: str = "") -> PoseEstimate:
    return get_bot_pose_estimate(limelight_name, "botpose_wpiblue")

def get_bot_pose_estimate_wpi_red(limelight_name: str = "") -> PoseEstimate:
    return get_bot_pose_estimate(limelight_name, "botpose_wpired")

INVALID_TARGET = 0.0

class SingleTargetingResultClass:
    def __init__(self):
        self.m_TargetXPixels = INVALID_TARGET
        self.m_TargetYPixels = INVALID_TARGET
        self.m_TargetXNormalized = INVALID_TARGET
        self.m_TargetYNormalized = INVALID_TARGET
        self.m_TargetXNormalizedCrosshairAdjusted = INVALID_TARGET
        self.m_TargetYNormalizedCrosshairAdjusted = INVALID_TARGET
        self.m_TargetXDegreesCrosshairAdjusted = INVALID_TARGET
        self.m_TargetYDegreesCrosshairAdjusted = INVALID_TARGET
        self.m_TargetAreaPixels = INVALID_TARGET
        self.m_TargetAreaNormalized = INVALID_TARGET
        self.m_TargetAreaNormalizedPercentage = INVALID_TARGET
        self.m_timeStamp = -1.0
        self.m_latency = 0
        self.m_pipelineIndex = -1.0
        self.m_TargetCorners = []
        self.m_CAMERATransform6DTARGETSPACE = []
        self.m_TargetTransform6DCAMERASPACE = []
        self.m_TargetTransform6DROBOTSPACE = []
        self.m_ROBOTTransform6DTARGETSPACE = []
        self.m_ROBOTTransform6DFIELDSPACE = []
        self.m_CAMERATransform6DROBOTSPACE = []

class RetroreflectiveResultClass(SingleTargetingResultClass):
    def __init__(self):
        super().__init__()

class FiducialResultClass(SingleTargetingResultClass):
    def __init__(self):
        super().__init__()
        self.m_fiducialID = 0
        self.m_family = ""

class DetectionResultClass(SingleTargetingResultClass):
    def __init__(self):
        super().__init__()
        self.m_classID = -1
        self.m_className = ""
        self.m_confidence = 0.0

class ClassificationResultClass(SingleTargetingResultClass):
    def __init__(self):
        super().__init__()
        self.m_classID = -1
        self.m_className = ""
        self.m_confidence = 0.0

class VisionResultsClass:
    def __init__(self):
        self.RetroResults = []
        self.FiducialResults = []
        self.DetectionResults = []
        self.ClassificationResults = []
        self.m_timeStamp = -1.0
        self.m_latencyPipeline = 0.0
        self.m_latencyCapture = 0.0
        self.m_latencyJSON = 0.0
        self.m_pipelineIndex = -1.0
        self.valid = 0
        self.botPose = [0.0] * 6
        self.botPose_wpired = [0.0] * 6
        self.botPose_wpiblue = [0.0] * 6

    def clear(self):
        self.RetroResults.clear()
        self.FiducialResults.clear()
        self.DetectionResults.clear()
        self.ClassificationResults.clear()
        self.botPose = [0.0] * 6
        self.botPose_wpired = [0.0] * 6
        self.botPose_wpiblue = [0.0] * 6
        self.m_timeStamp = -1.0
        self.m_latencyPipeline = 0.0
        self.m_pipelineIndex = -1.0

class LimelightResultsClass:
    def __init__(self):
        self.targetingResults = VisionResultsClass()


class InternalKeys:
    _key_timestamp = "ts"
    _key_latency_pipeline = "tl"
    _key_latency_capture = "cl"

    _key_pipelineIndex = "pID"
    _key_TargetXDegrees = "txdr"
    _key_TargetYDegrees = "tydr"
    _key_TargetXNormalized = "txnr"
    _key_TargetYNormalized = "tynr"
    _key_TargetXPixels = "txp"
    _key_TargetYPixels = "typ"

    _key_TargetXDegreesCrosshair = "tx"
    _key_TargetYDegreesCrosshair = "ty"
    _key_TargetXNormalizedCrosshair = "txn"
    _key_TargetYNormalizedCrosshair = "tyn"
    _key_TargetAreaNormalized = "ta"
    _key_TargetAreaPixels = "tap"
    _key_className = "class"
    _key_classID = "classID"
    _key_confidence = "conf"
    _key_fiducialID = "fID"
    _key_corners = "pts"
    _key_transformCAMERAPOSE_TARGETSPACE = "t6c_ts"
    _key_transformCAMERAPOSE_ROBOTSPACE = "t6c_rs"

    _key_transformTARGETPOSE_CAMERASPACE = "t6t_cs"
    _key_transformROBOTPOSE_TARGETSPACE = "t6r_ts"
    _key_transformTARGETPOSE_ROBOTSPACE = "t6t_rs"

    _key_botpose = "botpose"
    _key_botpose_wpiblue = "botpose_wpiblue"
    _key_botpose_wpired = "botpose_wpired"

    _key_transformROBOTPOSE_FIELDSPACE = "t6r_fs"
    _key_skew = "skew"
    _key_ffamily = "fam"
    _key_colorRGB = "cRGB"
    _key_colorHSV = "cHSV"

def phone_home():
    try:
        # Create socket (same as before)
        sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Set broadcast option (same as before)
        broadcast = 1
        if sockfd.setsockopt(sockfd, socket.SOL_SOCKET, socket.SO_BROADCAST, broadcast, 1) < 0:
            raise RuntimeError("Error setting Broadcast option")

        # Set socket to non-blocking (using socket.setblocking)
        sockfd.setblocking(False)

        # Prepare message and address
        msg = "LLPhoneHome"
        servaddr = (socket.inet_pton(socket.AF_INET, "255.255.255.255"), 5809)  # Tuple for address

        # Send message using sendto from socket module
        sockfd.sendto(sockfd, msg.encode(), 0, servaddr)  # Encode message for sending

        with sockfd:  # Use with statement for resource management

            # Receive data
            receiveData = b''
            while True:
                try:
                    data, addr = sockfd.recvfrom(sockfd, 1024)
                    receiveData += data
                    if not data:
                        break
                except BlockingIOError:
                    break  # No data received (non-blocking)

            if receiveData:
                # Process received data
                print(f"Received response: {receiveData.decode()}")

    except (OSError, socket.error) as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    phone_home()

def SetupPortForwarding(limelightName):
    portForwarder = PortForwarder.getInstance()
    
    for port in range(5800, 5810):
        portForwarder.add(port, sanitize_name(limelightName), port)

def SafeJSONAccess(jsonData, key, defaultValue):
    try:
        return jsonData[key]
    except KeyError:
        return defaultValue
    except Exception:
        return defaultValue

def from_json(j: wpiutil.json, t: RetroreflectiveResultClass):
    defaultValueVector = [0.0] * 6
    t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess(j, InternalKeys._key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector)
    t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess(j, InternalKeys._key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector)
    t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess(j, InternalKeys._key_transformTARGETPOSE_CAMERASPACE, defaultValueVector)
    t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess(j, InternalKeys._key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector)
    t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess(j, InternalKeys._key_transformROBOTPOSE_TARGETSPACE, defaultValueVector)
    t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess(j, InternalKeys._key_transformROBOTPOSE_FIELDSPACE, defaultValueVector)

    t.m_TargetXPixels = SafeJSONAccess(j, InternalKeys._key_TargetXPixels, 0.0)
    t.m_TargetYPixels = SafeJSONAccess(j, InternalKeys._key_TargetYPixels, 0.0)
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetXDegreesCrosshair, 0.0)
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetYDegreesCrosshair, 0.0)
    t.m_TargetAreaNormalized = SafeJSONAccess(j, InternalKeys._key_TargetAreaNormalized, 0.0)
    t.m_TargetCorners = SafeJSONAccess(j, InternalKeys._key_corners, [])

def from_json_FiducialResultClass(j: wpiutil.json, t: FiducialResultClass):
    defaultValueVector = [0.0] * 6
    t.m_family = SafeJSONAccess(j, InternalKeys._key_ffamily, "")
    t.m_fiducialID = SafeJSONAccess(j, InternalKeys._key_fiducialID, 0)
    t.m_CAMERATransform6DTARGETSPACE = SafeJSONAccess(j, InternalKeys._key_transformCAMERAPOSE_TARGETSPACE, defaultValueVector)
    t.m_CAMERATransform6DROBOTSPACE = SafeJSONAccess(j, InternalKeys._key_transformCAMERAPOSE_ROBOTSPACE, defaultValueVector)
    t.m_TargetTransform6DCAMERASPACE = SafeJSONAccess(j, InternalKeys._key_transformTARGETPOSE_CAMERASPACE, defaultValueVector)
    t.m_TargetTransform6DROBOTSPACE = SafeJSONAccess(j, InternalKeys._key_transformTARGETPOSE_ROBOTSPACE, defaultValueVector)
    t.m_ROBOTTransform6DTARGETSPACE = SafeJSONAccess(j, InternalKeys._key_transformROBOTPOSE_TARGETSPACE, defaultValueVector)
    t.m_ROBOTTransform6DFIELDSPACE = SafeJSONAccess(j, InternalKeys._key_transformROBOTPOSE_FIELDSPACE, defaultValueVector)
    t.m_TargetXPixels = SafeJSONAccess(j, InternalKeys._key_TargetXPixels, 0)
    t.m_TargetYPixels = SafeJSONAccess(j, InternalKeys._key_TargetYPixels, 0)
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetXDegreesCrosshair, 0)
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetYDegreesCrosshair, 0)
    t.m_TargetAreaNormalized = SafeJSONAccess(j, InternalKeys._key_TargetAreaNormalized, 0)
    t.m_TargetCorners = SafeJSONAccess(j, InternalKeys._key_corners, [])

def from_json_DetectionResultClass(j: wpiutil.json, t: DetectionResultClass):
    t.m_confidence = SafeJSONAccess(j, InternalKeys._key_confidence, 0)
    t.m_classID = SafeJSONAccess(j, InternalKeys._key_classID, 0)
    t.m_className = SafeJSONAccess(j, InternalKeys._key_className, "")
    t.m_TargetXPixels = SafeJSONAccess(j, InternalKeys._key_TargetXPixels, 0)
    t.m_TargetYPixels = SafeJSONAccess(j, InternalKeys._key_TargetYPixels, 0)
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetXDegreesCrosshair, 0)
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetYDegreesCrosshair, 0)
    t.m_TargetAreaNormalized = SafeJSONAccess(j, InternalKeys._key_TargetAreaNormalized, 0)
    t.m_TargetCorners = SafeJSONAccess(j, InternalKeys._key_corners, [])

def from_json_ClassificationResultClass(j: wpiutil.json, t: ClassificationResultClass):
    t.m_confidence = SafeJSONAccess(j, InternalKeys._key_confidence, 0)
    t.m_classID = SafeJSONAccess(j, InternalKeys._key_classID, 0)
    t.m_className = SafeJSONAccess(j, InternalKeys._key_className, "")
    t.m_TargetXPixels = SafeJSONAccess(j, InternalKeys._key_TargetXPixels, 0)
    t.m_TargetYPixels = SafeJSONAccess(j, InternalKeys._key_TargetYPixels, 0)
    t.m_TargetXDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetXDegreesCrosshair, 0)
    t.m_TargetYDegreesCrosshairAdjusted = SafeJSONAccess(j, InternalKeys._key_TargetYDegreesCrosshair, 0)
    t.m_TargetAreaNormalized = SafeJSONAccess(j, InternalKeys._key_TargetAreaNormalized, 0)
    t.m_TargetCorners = SafeJSONAccess(j, InternalKeys._key_corners, [])

def from_json_VisionResultsClass(j: wpiutil.json, t: VisionResultsClass):
    t.m_timeStamp = SafeJSONAccess(j, InternalKeys._key_timestamp, 0.0)
    t.m_latencyPipeline = SafeJSONAccess(j, InternalKeys._key_latency_pipeline, 0.0)
    t.m_latencyCapture = SafeJSONAccess(j, InternalKeys._key_latency_capture, 0.0)
    t.m_pipelineIndex = SafeJSONAccess(j, InternalKeys._key_pipelineIndex, 0.0)
    t.valid = SafeJSONAccess(j, "v", 0)
    
    defaultVector = []
    t.botPose = SafeJSONAccess(j, InternalKeys._key_botpose, defaultVector)
    t.botPose_wpired = SafeJSONAccess(j, InternalKeys._key_botpose_wpired, defaultVector)
    t.botPose_wpiblue = SafeJSONAccess(j, InternalKeys._key_botpose_wpiblue, defaultVector)
    
    t.RetroResults = SafeJSONAccess(j, "Retro", [])
    t.FiducialResults = SafeJSONAccess(j, "Fiducial", [])
    t.DetectionResults = SafeJSONAccess(j, "Detector", [])
    t.ClassificationResults = SafeJSONAccess(j, "Detector", [])

def from_json_LimelightResultsClass(j: wpiutil.json, t: LimelightResultsClass):
    t.targetingResults = SafeJSONAccess(j, "Results", VisionResultsClass())

def getLatestResults(limelightName="", profile=False):
    start = time.time()
    jsonString = get_json_dump(limelightName)
    data = json 
    try:
        data = json.loads(jsonString)
    except Exception as e:
        return LimelightResultsClass()
    
    end = time.time()
    millis = (end - start) * 1000
    
    try:
        out = data["Results"]
        out["m_latencyJSON"] = millis
        if profile:
            print("lljson:", millis)
        return out
    except:
        return LimelightResultsClass()
