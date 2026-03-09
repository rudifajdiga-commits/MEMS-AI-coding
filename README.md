# ESP32 IMU Dashboard with WebGL

A modern, high-performance IMU telemetry dashboard for ESP32-S3 using WebGL and WebSockets.

## Features
- **High-Poly 3D Orientation**: Realistic human model (`FinalBaseMesh.glb`) rendered via WebGL (Three.js).
- **Real-time Telemetry**: Low-latency data streaming using WebSockets.
- **Modern UI**: Aerospace-style dark theme dashboard with live Euler angles and Accelerometer graphs.
- **Calibrate Zero**: Instantly reset orientation to 0,0,0 via phone UI using quaternion math.
- **4MB Flash Optimized**: Assets are Gzip-compressed and embedded directly into flash.
- **SoftAP Support**: Works 100% offline (creates its own WiFi network).

## Technical Implementation Details
- **Rendering Engine**: Three.js (via CDN) + GLTFLoader.
- **Data Pipeline**: LSM6DSV16X IMU -> Madgwick AHRS Filter (ESP32) -> Quaternions -> WebSocket -> WebGL (Browser).
- **Storage**: Custom partition table (`partitions.csv`) to maximize app space for 3D assets.
- **Compression**: 3D model compressed from 3.5MB to 1.0MB using Gzip to fit within 4MB total flash.

## Deployment
1. Build & Flash:
   ```powershell
   idf.py build
   idf.py flash
   ```
2. Connect to WiFi SSID: `ESP32_IMU` (Password: `12345678`).
3. Open `http://192.168.4.1` in your mobile browser.
4. *Note: Keep mobile data active to load the 3D engine from CDN.*

## Changelog (March 2026)
- **Dependency Cleanup**: Removed unused `led_strip` and `managed_components`.
- **Compilation Fixes**: Corrected IP address structure and enabled WebSocket support in `sdkconfig`.
- **UI Redesign**: Implemented a futuristic "Astonishing" engineering dashboard.
- **3D Evolution**: From simple Cube (Canvas 2D) -> Humanoid Boxes -> High-Poly GLB Model (WebGL).
- **Calibration System**: Added client-side quaternion tare logic.
- **Flash Optimization**: Implemented Gzip-encoding for embedded files and custom 4MB partition layout.
