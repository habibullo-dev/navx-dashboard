import cv2
import asyncio
import websockets
import json
import time
import torch
import threading
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

# --- CONFIGURATION ---
# The IP of your Raspberry Pi (from your HTML snippet)
PI_STREAM_URL = "http://172.20.10.10:8080/?action=stream"  # MJPEG stream URL
WEBSOCKET_PORT = 8765  # WebSocket port for dashboard

# Target width for inference (maintain aspect)
TARGET_INFERENCE_WIDTH = 640

# Initial frame skip (process 1 in every N frames); may adapt
INITIAL_FRAME_SKIP = 3

# Adaptive skip bounds
MIN_FRAME_SKIP = 2
MAX_FRAME_SKIP = 6

# Reconnection settings
MAX_READ_FAILURES = 10
RECONNECT_DELAY_SEC = 2

# Confidence threshold for reporting detections
CONF_THRESHOLD = 0.4

# Global State for ROS Data
latest_battery_pct = "--"
latest_nav_status = None

class BridgeNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')
        
        # Subscribe to Battery
        self.sub_batt = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        # Subscribe to Navigation Status
        self.sub_nav = self.create_subscription(
            String,
            '/nav_status',
            self.nav_callback,
            10
        )
        
        self.get_logger().info("Bridge Node Started. Listening for /battery_state and /nav_status")

    def battery_callback(self, msg: BatteryState):
        global latest_battery_pct
        try:
            latest_battery_pct = int(msg.percentage)
        except:
            pass

    def nav_callback(self, msg: String):
        global latest_nav_status
        try:
            latest_nav_status = json.loads(msg.data)
        except:
            pass


# Load the lightweight YOLOv8 Nano model
# It will download 'yolov8n.pt' automatically on first run (~6MB)
print("Loading YOLOv8n model...")
device = "cuda" if torch.cuda.is_available() else "cpu"
model = YOLO("yolov8n.pt").to(device)


async def detection_handler(websocket):
    print("Client connected to AI stream!")

    def open_capture():
        cap_local = cv2.VideoCapture(PI_STREAM_URL, cv2.CAP_FFMPEG)
        cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap_local

    cap = None
    frame_count = 0
    effective_skip = INITIAL_FRAME_SKIP
    
    # Try to open initially
    cap = open_capture()
    
    try:
        while True:
            # --- 1. CAMERA HANDLING ---
            frame_resized = None
            detections = []
            infer_ms = 0
            
            if cap is None or not cap.isOpened():
                # Try to reconnect every 2 seconds (approx 60 frames at 30fps loop)
                if frame_count % 60 == 0:
                    print("Attempting to connect to camera...")
                    if cap: cap.release()
                    cap = open_capture()
            
            if cap and cap.isOpened():
                success, frame = cap.read()
                if not success or frame is None:
                    # Read failed
                    if frame_count % 30 == 0:
                        print("Stream read failed, retrying...")
                    cap.release()
                    cap = None
                else:
                    # Frame valid
                    if frame_count % effective_skip == 0:
                        # Resize
                        h, w = frame.shape[:2]
                        if w != TARGET_INFERENCE_WIDTH:
                            new_h = int(h * (TARGET_INFERENCE_WIDTH / w))
                            frame_resized = cv2.resize(frame, (TARGET_INFERENCE_WIDTH, new_h), interpolation=cv2.INTER_AREA)
                        else:
                            frame_resized = frame

                        # Inference
                        t0 = time.perf_counter()
                        results = model(frame_resized, stream=True, verbose=False, half=(device == "cuda"))
                        infer_ms = (time.perf_counter() - t0) * 1000.0

                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                x1, y1, x2, y2 = box.xyxyn[0].tolist()
                                conf = float(box.conf[0])
                                if conf < CONF_THRESHOLD:
                                    continue
                                cls = int(box.cls[0])
                                label = model.names.get(cls, str(cls))
                                detections.append({
                                    "label": label,
                                    "conf": round(conf, 3),
                                    "bbox": [x1, y1, x2, y2]
                                })
                        
                        # Adaptive skip
                        if infer_ms > 50 and effective_skip < MAX_FRAME_SKIP:
                            effective_skip += 1
                        elif infer_ms < 30 and effective_skip > MIN_FRAME_SKIP:
                            effective_skip -= 1

            # --- 2. SEND PAYLOAD ---
            # Always send payload, even if camera is down (detections will be empty)
            payload = {
                "timestamp": time.time(),
                "objects": detections,
                "infer_ms": round(infer_ms, 2),
                "battery": latest_battery_pct,
                "nav_status": latest_nav_status
            }
            
            if frame_resized is not None:
                payload["width"] = frame_resized.shape[1]
                payload["height"] = frame_resized.shape[0]
            
            await websocket.send(json.dumps(payload))
            
            frame_count += 1
            await asyncio.sleep(0.01) # ~100Hz loop speed limit (logic only)

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        if cap: cap.release()

async def main():
    # Start ROS Node in a separate thread
    rclpy.init()
    node = BridgeNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    print(f"Starting AI WebSocket server on port {WEBSOCKET_PORT}...")
    print(f"Reading stream from: {PI_STREAM_URL}")
    
    async with websockets.serve(detection_handler, "0.0.0.0", WEBSOCKET_PORT):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())