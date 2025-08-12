import pyrealsense2 as rs
import numpy as np
import cv2
from flask import Flask, Response

app = Flask(__name__)

# Настройка RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Запуск потока
pipeline.start(config)

def generate_frames():
    try:
        while True:
            # Получаем кадр
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Конвертируем в numpy-массив
            frame = np.asanyarray(color_frame.get_data())
            
            # Кодируем в JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            
            # Формируем MJPEG-поток
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    finally:
        pipeline.stop()

@app.route('/')
def index():
    return """
    <html>
        <head>
            <title>RealSense Live Stream</title>
        </head>
        <body>
            <h1>RealSense Live Stream</h1>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def main():
    app.run(host='192.168.123.162', port=5010, threaded=True)


if __name__ == '__main__':
    main()