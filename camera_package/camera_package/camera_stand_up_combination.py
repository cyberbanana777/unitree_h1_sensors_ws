# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Flask-сервер для трансляции двух видеопотоков с камеры Intel RealSense. Обеспечивает:
  - Цветное видео (640×480, 30 FPS, формат BGR8)
  - Данные глубины (640×480, 30 FPS, формат Z16) с цветовой визуализацией
  - Параллельное отображение обоих потоков через веб интерфейс

Техническая реализация:
- Использует pyrealsense2 для управления камерой и захвата кадров
- Реализует MJPEG-трансляцию через multipart-ответы Flask

Требования:
- Камера Intel RealSense (рекомендуется серия D400)
- Библиотеки: pyrealsense2, OpenCV, Flask

Сервер по умолчанию работает в многопоточном режиме (хост: 192.168.123.162, порт: 5010)

ANNOTATION
Flask-server for broadcasting two video streams from the Intel RealSense camera. Provides:
 - Color video (640×480, 30 FPS, BGR8 format)
 - Depth data (640×480, 30 FPS, Z16 format) with color visualization
 - Parallel display of both streams via web interface

Technical implementation:
- Uses pyrealsense2 to control the camera and capture frames
- Realizes MJPEG streaming via Flask multipart responses

Requirements:
- Intel RealSense camera (D400 series recommended)
- Libraries: pyrealsense2, OpenCV, Flask

The server runs in multithreaded mode by default (host: 192.168.123.162, port: 5010)
'''



import pyrealsense2 as rs
import numpy as np
import cv2
from flask import Flask, Response


# ==============================================================================
# RealSense Camera Configuration
# ==============================================================================

app = Flask(__name__)

# Initialize RealSense pipeline and configuration
pipeline = rs.pipeline()
config  = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

# Create depth colorizer for visualization
colorizer = rs.colorizer()


# ==============================================================================
# Frame Generation Functions
# ==============================================================================

def generate_color_frames():
    """Generator function that continuously yields color frames as JPEG."""
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            # Convert frame to numpy array and encode as JPEG
            color_image = np.asanyarray(color_frame.get_data())
            ret, buffer = cv2.imencode('.jpg', color_image)
            frame = buffer.tobytes()
            
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + 
                frame + 
                b'\r\n'
            )

    finally:
        pipeline.stop()


def generate_depth_frames():
    """Generator function that continuously yields depth frames (colorized) as JPEG."""
    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue
            
            # Colorize depth frame and encode as JPEG
            depth_color_frame = colorizer.colorize(depth_frame)
            depth_image = np.asanyarray(depth_color_frame.get_data())
            ret, buffer = cv2.imencode('.jpg', depth_image)
            frame = buffer.tobytes()
            
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + 
                frame + 
                b'\r\n'
            )

    finally:
        pipeline.stop()


# ==============================================================================
# Flask Routes
# ==============================================================================

@app.route('/')
def index():
    """Main page that displays both color and depth streams."""
    return """
    <html>
        <head>
            <title>RealSense Dual Stream</title>
            <style>
                .container {
                    display: flex;
                    flex-direction: column;
                    align-items: center;
                }
                .stream-container {
                    display: flex;
                    justify-content: center;
                    margin-bottom: 20px;
                }
                .stream-box {
                    margin: 0 10px;
                    text-align: center;
                }
                h1 {
                    text-align: center;
                    color: #333;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>Intel RealSense Dual Stream</h1>
                
                <div class="stream-container">
                    <div class="stream-box">
                        <h2>Color Stream</h2>
                        <img src="/color_feed" width="640" height="480">
                    </div>
                    <div class="stream-box">
                        <h2>Depth Stream</h2>
                        <img src="/depth_feed" width="640" height="480">
                    </div>
                </div>
            </div>
        </body>
    </html>
    """


@app.route('/color_feed')
def color_feed():
    """Endpoint for color video feed."""
    return Response(
        generate_color_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/depth_feed')
def depth_feed():
    """Endpoint for depth video feed."""
    return Response(
        generate_depth_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Start the Flask web server."""
    app.run(host='192.168.123.162', port=5010, threaded=True)


if __name__ == '__main__':
    main()