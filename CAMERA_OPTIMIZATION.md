# Camera & Detection Optimization Guide

This document captures recommended settings and software-side optimizations to achieve low latency, efficient resource usage, and stable object detection for your TurtleBot3 POV streaming pipeline.

## Goals
- Minimize end-to-end latency from capture to browser overlay
- Reduce CPU/GPU load without hurting detection quality
- Improve detection confidence via cleaner frames (lighting + exposure)
- Provide reproducible commands for tuning and validation

## Components Overview
| Component | Role | Optimization Focus |
|----------|------|--------------------|
| USB Camera (Razer Kiyo) | Frame source (UVC) | Resolution/FPS, MJPEG output, exposure, focus |
| mjpg-streamer | HTTP MJPEG stream to browser & detection server | Frame size/FPS, stability |
| detection_server.py | Pulls stream, runs YOLO, sends JSON via WebSocket | Frame skip, resize, GPU usage |
| Browser (camera.html) | Renders MJPEG + overlays | Efficient canvas redraw, minimal payload |

## Recommended Operating Modes
| Mode | Camera Output | Inference Resize | FPS | Skip | Use Case |
|------|---------------|------------------|-----|------|----------|
| Balanced | 800x600 MJPEG | 640 width | 25 | 2-3 | Default POV + detection |
| Low Bandwidth | 640x480 MJPEG | 640 width | 15-20 | 2 | Constrained network |
| High Clarity View | 1280x720 MJPEG | 640 width | 30 | 3 | Public demo / HD viewer |
| High Motion | 720p MJPEG | 640 width | 30 | 2 | Fast navigation scenes |

## Camera Enumeration & Formats
List devices and formats:
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```
Select an *MJPG* format at your target resolution. Prefer MJPEG over raw YUYV to reduce CPU.

## Launch mjpg-streamer (Example)
```bash
mjpg_streamer \
  -i "input_uvc.so -d /dev/video0 -r 800x600 -f 25" \
  -o "output_http.so -p 8080 -w /usr/share/mjpg-streamer/www"
```
Adjust `-r` and `-f` per mode. Avoid 1080p unless absolutely required.

## Useful v4l2 Controls (Check availability first)
List controls:
```bash
v4l2-ctl -d /dev/video0 --list-ctrls
```
Set manual parameters (values may vary by device):
```bash
# Exposure (manual)
v4l2-ctl -d /dev/video0 -c exposure_auto=1
v4l2-ctl -d /dev/video0 -c exposure_absolute=250  # Tune lower for less motion blur

# White balance (manual)
v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 -c white_balance_temperature=4500

# Focus (if supported)
v4l2-ctl -d /dev/video0 -c focus_auto=0
v4l2-ctl -d /dev/video0 -c focus_absolute=15

# Gain (reduce noise if lighting good)
v4l2-ctl -d /dev/video0 -c gain=1
```
Use the ring light at the lowest brightness that yields crisp edges.

## Detection Server Optimizations
1. **FFMPEG Backend & Low Buffer**: Reduces latency.
2. **Frame Resize Before Inference**: Stable 640 px width for YOLO.
3. **Adaptive Frame Skipping**: Tune skip based on inference time.
4. **Inference Timing Metrics**: Send `infer_ms` to UI.
5. **GPU / Half Precision (if CUDA)**: Lower inference latency.
6. **Reconnect Logic**: Recover from transient stream drops.

## Metrics To Monitor
- `infer_ms` (per processed frame)
- Effective `skip` value
- Average detection confidence
- CPU usage (`top`, `htop`)
- Network throughput (if remote viewers increase)

## Validation Procedure
1. Start mjpg-streamer with target resolution/FPS.
2. Run detection server; gather baseline metrics (~2 min).
3. Apply tuning (exposure, gain, resize, skip).
4. Compare average `infer_ms` and detection confidence distribution.
5. Iterate until latency & quality stabilize.

## Potential Next-Level Improvements
| Improvement | Benefit | Complexity |
|------------|---------|-----------|
| RTSP/H.264 (v4l2rtspserver) | Lower bandwidth | Medium |
| WebRTC Gateway (mediamtx) | Browser-native efficient stream | Medium/High |
| ONNX + INT8 Quantization | Faster CPU inference | Medium |
| Dual-Stream Strategy | HD viewer + low-res inference | Low/Medium |

## Systemd Service Examples
`/etc/systemd/system/mjpg-streamer.service`:
```ini
[Unit]
Description=MJPG Streamer
After=network.target

[Service]
ExecStart=/usr/bin/mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 800x600 -f 25" -o "output_http.so -p 8080 -w /usr/share/mjpg-streamer/www"
Restart=always
User=www-data
Group=www-data

[Install]
WantedBy=multi-user.target
```
Enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now mjpg-streamer
```

## Troubleshooting
| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| High latency spikes | Large internal buffer | Set CAP_PROP_BUFFERSIZE=1; lower FPS |
| CPU usage high | Raw YUYV capture | Switch to MJPEG format |
| Detection confidence drops | Underexposed frames | Adjust exposure/ring light/gain |
| Stream freezes | Network hiccup or camera reset | Reconnect logic in capture loop |
| Browser lag | Oversized resolution | Reduce viewer stream size |

## Quick Command Recap
```bash
# List formats
v4l2-ctl -d /dev/video0 --list-formats-ext

# Start MJPEG stream
mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 800x600 -f 25" -o "output_http.so -p 8080 -w /usr/share/mjpg-streamer/www"

# Basic tuning
v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=250
v4l2-ctl -d /dev/video0 -c gain=1
```

## Next Steps Implemented
- Patch `detection_server.py` (timing, adaptive skip, resize, reconnect, GPU usage) — see file for updated logic.

## Summary
A higher-end camera produces better input; the real gains come from disciplined configuration: moderate resolution, consistent lighting, MJPEG output, controlled frame skipping, and robust capture logic. These changes keep latency low and detection quality high without overloading CPU/GPU.

---
*Update this document as you measure real metrics when hardware becomes available.*
