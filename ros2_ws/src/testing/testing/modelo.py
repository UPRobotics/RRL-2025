import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
import numpy as np
import time
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# ✅ Classes
CLASSES = [
    'hydrant', 'aquatic_toxicity', 'fire_extinguisher', 'high_voltage',
    'exclamation_mark', 'healthy_hazard', 'biohazard', 'radioactive',
    'fire_hose_reel', 'stairway', 'emergency_exit', 'low_temp',
    'corrosive', 'non_flam_gas', 'magnetic_field', 'danger_of_death',
    'oxidizer', 'explosive', 'flammable', 'laser_radiation'
]

BLOB_PATH = "/home/chumbi/ros2_ws/modelo.blob"
INPUT_SIZE = 640
CONF_THRESHOLD = 0.3
IOU_THRESHOLD = 0.3

class OakDS2Node(Node):
    def __init__(self):
        super().__init__('oak_d_s2_node')
        self.get_logger().info('Initializing OAK-D S2 ROS 2 Node')

        # ROS 2 Publishers
        self.image_pub = self.create_publisher(Image, '/oak_d_s2/rgb/image', 10)
        self.detections_pub = self.create_publisher(String, '/oak_d_s2/detections', 10)
        self.bridge = CvBridge()

        # 📌 IoU Function
        self.iou = lambda box1, box2: (
            (x1 := max(box1[0], box2[0])) and
            (y1 := max(box1[1], box2[1])) and
            (x2 := min(box1[2], box2[2])) and
            (y2 := min(box1[3], box2[3])) and
            (inter_area := max(0, x2 - x1) * max(0, y2 - y1)) and
            (box1_area := (box1[2] - box1[0]) * (box1[3] - box1[1])) and
            (box2_area := (box2[2] - box2[0]) * (box2[3] - box2[1])) and
            (union_area := box1_area + box2_area - inter_area) and
            (inter_area / union_area if union_area > 0 else 0)
        )

        # 📌 NMS Function
        def non_max_suppression(detections):
            detections = sorted(detections, key=lambda x: x['conf'], reverse=True)
            filtered = []
            while detections:
                best = detections.pop(0)
                filtered.append(best)
                detections = [
                    d for d in detections
                    if d['class_id'] != best['class_id'] or self.iou(d['bbox'], best['bbox']) < IOU_THRESHOLD
                ]
            return filtered
        self.nms = non_max_suppression

        # 🎥 Pipeline Setup (Identical to Original, with FPS set to 10)
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(INPUT_SIZE, INPUT_SIZE)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam_rgb.setFps(10)  # Set camera to 10 FPS

        nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(BLOB_PATH)
        nn.setConfidenceThreshold(CONF_THRESHOLD)
        nn.setNumClasses(len(CLASSES))
        nn.setCoordinateSize(4)

        anchors = [10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]
        nn.setAnchors(anchors)
        nn.setAnchorMasks({
            "side80": [0, 1, 2],
            "side40": [3, 4, 5],
            "side20": [6, 7, 8]
        })
        nn.setIouThreshold(IOU_THRESHOLD)
        cam_rgb.preview.link(nn.input)

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_nn = self.pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        nn.out.link(xout_nn.input)

        # 🚀 Initialize Device
        try:
            self.device = dai.Device(self.pipeline)
            self.q_rgb = self.device.getOutputQueue("rgb", 8, True)
            self.q_nn = self.device.getOutputQueue("nn", 8, True)
            self.get_logger().info('OAK-D S2 Camera Connected')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize OAK-D S2: {str(e)}')
            raise

        # Start Processing Timer at 10 Hz (0.1 seconds per frame)
        self.timer = self.create_timer(1.0 / 10, self.process_frame)

    def process_frame(self):
        try:
            start_time = time.time()
            in_rgb = self.q_rgb.get()
            in_nn = self.q_nn.get()

            if in_rgb is None or in_nn is None:
                self.get_logger().warn('Received None from RGB or NN queue, skipping frame')
                return

            frame = in_rgb.getCvFrame()
            detections = in_nn.detections

            self.get_logger().info(f"Detections: {len(detections)}, Confidences: {[det.confidence for det in detections]}")

            processed_detections = []
            for det in detections:
                if det.confidence < CONF_THRESHOLD or det.label >= len(CLASSES):
                    self.get_logger().warn(f'Skipping invalid detection: label={det.label}, confidence={det.confidence}')
                    continue
                x1 = int(det.xmin * frame.shape[1])
                y1 = int(det.ymin * frame.shape[0])
                x2 = int(det.xmax * frame.shape[1])
                y2 = int(det.ymax * frame.shape[0])
                if x1 >= x2 or y1 >= y2 or x1 < 0 or y1 < 0 or x2 > frame.shape[1] or y2 > frame.shape[0]:
                    self.get_logger().warn(f'Invalid bounding box: [{x1}, {y1}, {x2}, {y2}]')
                    continue
                processed_detections.append({
                    'class_id': det.label,
                    'conf': det.confidence,
                    'bbox': [x1, y1, x2, y2]
                })

            processed_detections = self.nms(processed_detections)

            # Prepare detection message as JSON
            detection_data = []
            drawn_text_positions = []
            for det in processed_detections:
                x1, y1, x2, y2 = det['bbox']
                class_name = CLASSES[det['class_id']]
                conf = det['conf']
                label_text = f"{class_name} {conf:.2f}"

                text_pos = (x1, y1 - 10)
                if any(np.linalg.norm(np.array(text_pos) - np.array(pos)) < 20 for pos in drawn_text_positions):
                    continue

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (64, 0, 255), 2)
                drawn_text_positions.append(text_pos)

                detection_data.append({
                    'class_name': class_name,
                    'confidence': float(conf),
                    'bbox': [x1, y1, x2, y2]
                })

            # Publish detections as JSON string
            try:
                detection_msg = String()
                detection_msg.data = json.dumps(detection_data)
                self.detections_pub.publish(detection_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish detections: {str(e)}')

            # Publish Image
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'oak_d_s2_rgb'
                self.image_pub.publish(image_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {str(e)}')

            self.get_logger().info(f"Frame time: {time.time() - start_time:.3f} seconds")

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def destroy_node(self):
        super().destroy_node()
        try:
            self.device.close()
            self.get_logger().info('OAK-D S2 Device Closed')
        except Exception as e:
            self.get_logger().error(f'Error closing device: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OakDS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()