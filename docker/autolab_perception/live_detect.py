import roslibpy
import cv2
import numpy as np
import base64
import json
from ultralytics import YOLO

# 1. Load the model using your confirmed path
model =YOLO('weights/detection/wellplate_detect.pt')
seg_model = YOLO('weights/segmentation/wellplate_seg.pt')

# 2. Connect to the ROS environment
client = roslibpy.Ros(host='172.26.230.95', port=9090)

def callback(message):
    try:
        # Decode image from ROS message
        img_bytes = base64.b64decode(message['data'])
        nparr = np.frombuffer(img_bytes, dtype=np.uint8)
        img = nparr.reshape((message['height'], message['width'], -1))

        # FIX: Handle BGRA/RGBA to BGR conversion for YOLO compatibility
        if img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        
        # 3. Inference
        results = model.predict(source=img, conf=0.5, verbose=False)

        # 4. Process detections
        if len(results) > 0 and len(results[0].boxes) > 0:
            names = model.names
            detected = []
            for box in results[0].boxes:
                label = names[int(box.cls)]
                coords = box.xywh[0].tolist() # [x_center, y_center, width, height]
                detected.append({"label": label, "pose_2d": coords})
            
            print(f"Detected: {detected}")
            
            # Visualization
            annotated_frame = results[0].plot()
            cv2.imshow("Autolab Real-Time", annotated_frame)
            cv2.waitKey(1)

    except Exception as e:
        print(f"Processing error: {e}")

# 5. Subscribe to the ZED RGB stream
client.run()
topic = '/zed/zed_node/rgb/image_rect_color'
listener = roslibpy.Topic(client, topic, 'sensor_msgs/Image')
listener.subscribe(callback)

print("Pipeline Active. Waiting for ZED data...")

try:
    while client.is_connected:
        pass
except KeyboardInterrupt:
    client.terminate()
    cv2.destroyAllWindows()
