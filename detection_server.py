import cv2
import asyncio
import websockets
import json
import time
import torch
from ultralytics import YOLO

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

# Load the lightweight YOLOv8 Nano model
# It will download 'yolov8n.pt' automatically on first run (~6MB)
print("Loading YOLOv8n model...")
device = "cuda" if torch.cuda.is_available() else "cpu"
model = YOLO("yolov8n.pt").to(device)
if device == "cuda":
    try:
        model.model.half()
        print("Model loaded in half precision on CUDA for speed.")
    except Exception:
        print("Half precision not applied; continuing in FP32.")

async def detection_handler(websocket):
    print("Client connected to AI stream!")

    def open_capture():
        cap_local = cv2.VideoCapture(PI_STREAM_URL, cv2.CAP_FFMPEG)
        cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap_local

    cap = open_capture()
    if not cap.isOpened():
        print(f"Error: Could not open stream at {PI_STREAM_URL}")
        await websocket.send(json.dumps({"error": "Stream unavailable"}))
        return

    frame_count = 0
    effective_skip = INITIAL_FRAME_SKIP
    read_failures = 0

    try:
        while True:
            success, frame = cap.read()
            if not success or frame is None:
                read_failures += 1
                if read_failures >= MAX_READ_FAILURES:
                    print("Stream read failures exceeded threshold; attempting reconnect...")
                    cap.release()
                    await asyncio.sleep(RECONNECT_DELAY_SEC)
                    cap = open_capture()
                    read_failures = 0
                    continue
                await asyncio.sleep(0.01)
                continue

            read_failures = 0
            frame_count += 1

            if frame_count % effective_skip != 0:
                await asyncio.sleep(0.001)
                continue

            # Maintain aspect ratio when resizing
            h, w = frame.shape[:2]
            if w != TARGET_INFERENCE_WIDTH:
                new_h = int(h * (TARGET_INFERENCE_WIDTH / w))
                frame_resized = cv2.resize(frame, (TARGET_INFERENCE_WIDTH, new_h), interpolation=cv2.INTER_AREA)
            else:
                frame_resized = frame

            # Convert to proper dtype if using half precision on CUDA
            if device == "cuda" and getattr(model.model, 'half', None):
                # YOLO handles conversion internally; keep frame as uint8
                pass

            t0 = time.perf_counter()
            results = model(frame_resized, stream=True, verbose=False)
            infer_ms = (time.perf_counter() - t0) * 1000.0

            detections = []
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

            # Adaptive skip logic based on inference time
            if infer_ms > 50 and effective_skip < MAX_FRAME_SKIP:
                effective_skip += 1
            elif infer_ms < 30 and effective_skip > MIN_FRAME_SKIP:
                effective_skip -= 1

            payload = json.dumps({
                "timestamp": time.time(),
                "objects": detections,
                "infer_ms": round(infer_ms, 2),
                "skip": effective_skip,
                "width": frame_resized.shape[1],
                "height": frame_resized.shape[0]
            })
            await websocket.send(payload)
            await asyncio.sleep(0.001)

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        cap.release()

async def main():
    print(f"Starting AI WebSocket server on port {WEBSOCKET_PORT}...")
    print(f"Reading stream from: {PI_STREAM_URL}")
    async with websockets.serve(detection_handler, "localhost", WEBSOCKET_PORT):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())