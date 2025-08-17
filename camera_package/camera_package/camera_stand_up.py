# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Flask-сервер для трансляции видео с камеры Intel RealSense. Формирует MJPEG-поток 
с разрешением 640x480 (30 FPS) через HTTP. Включает:
- Захват и кодирование видео в реальном времени
- Веб-интерфейс для просмотра потока
- Поддержку одновременных подключений (threaded=True)
Требует камеру RealSense и библиотеки pyrealsense2. Запускается на указанном IP и порту.

ANNOTATION
Flask server for streaming video from Intel RealSense camera. Provides MJPEG stream 
at 640x480 resolution (30 FPS) via HTTP. Features:
- Real-time video capture and encoding
- Web interface for stream viewing
- Support for concurrent connections (threaded=True)
Requires RealSense camera and pyrealsense2 library. Runs on specified IP and port.
'''


import pyrealsense2 as rs
import numpy as np
import cv2
from flask import Flask, Response


# ==============================================================================
# RealSense Camera Initialization
# ==============================================================================

app = Flask(__name__)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color stream (640x480 resolution, BGR8 format, 30 FPS)
config.enable_stream(
    rs.stream.color, 
    width=640, 
    height=480, 
    format=rs.format.bgr8, 
    framerate=30
)

# Start the pipeline with configuration
pipeline.start(config)


# ==============================================================================
# Video Frame Generation
# ==============================================================================

def generate_frames():
    """Generator function that continuously captures and yields video frames as JPEG."""
    try:
        while True:
            # Wait for and retrieve frames from the camera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue  # Skip if no frame was received
            
            # Convert frame to numpy array for OpenCV processing
            frame = np.asanyarray(color_frame.get_data())
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            # Yield frame in MJPEG stream format
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + 
                frame_bytes + 
                b'\r\n'
            )

    finally:
        # Ensure pipeline is properly stopped on exit
        pipeline.stop()


# ==============================================================================
# Flask Web Application Routes
# ==============================================================================

@app.route('/')
def index():
    """Main page that displays the live video stream."""
    return """
    <html>
        <head>
            <title>RealSense Live Stream</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    text-align: center;
                    margin-top: 50px;
                }
                h1 {
                    color: #333;
                    margin-bottom: 30px;
                }
            </style>
        </head>
        <body>
            <h1>RealSense Live Stream</h1>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    """Video streaming route that serves the MJPEG feed."""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


# ==============================================================================
# Main Application Entry Point
# ==============================================================================

def main():
    """Start the Flask web server."""
    app.run(
        host='192.168.123.162',
        port=5010,
        threaded=True
    )


if __name__ == '__main__':
    main()