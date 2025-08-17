# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import pyrealsense2 as rs
import numpy as np
import cv2
from flask import Flask, Response

app = Flask(__name__)

# Настройка RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Запуск потока
pipeline.start(config)

# Создаем цветовую карту для глубины
colorizer = rs.colorizer()

def generate_combined_frames():
    try:
        while True:
            # Получаем кадры
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            
            # Конвертируем цветной кадр
            color_image = np.asanyarray(color_frame.get_data())
            
            # Конвертируем карту глубины в цветное изображение
            depth_color_frame = colorizer.colorize(depth_frame)
            depth_image = np.asanyarray(depth_color_frame.get_data())
            
            # Объединяем изображения горизонтально
            combined_image = np.hstack((color_image, depth_image))
            
            # Кодируем в JPEG
            ret, buffer = cv2.imencode('.jpg', combined_image)
            frame = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    finally:
        pipeline.stop()

def generate_color_frames():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            ret, buffer = cv2.imencode('.jpg', color_image)
            frame = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    finally:
        pipeline.stop()

def generate_depth_frames():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            
            depth_color_frame = colorizer.colorize(depth_frame)
            depth_image = np.asanyarray(depth_color_frame.get_data())
            ret, buffer = cv2.imencode('.jpg', depth_image)
            frame = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    finally:
        pipeline.stop()

@app.route('/')
def index():
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
                
                <div class="stream-container">
                    <h2>Combined View</h2>
                    <img src="/combined_feed" width="1280" height="480">
                </div>
            </div>
        </body>
    </html>
    """

@app.route('/color_feed')
def color_feed():
    return Response(
        generate_color_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/depth_feed')
def depth_feed():
    return Response(
        generate_depth_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/combined_feed')
def combined_feed():
    return Response(
        generate_combined_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def main():
    app.run(host='192.168.123.162', port=5010, threaded=True)
    

if __name__ == '__main__':
    main()