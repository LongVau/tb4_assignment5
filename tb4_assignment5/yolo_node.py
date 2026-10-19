"""Subscribe to TurtleBot 4 RGB images and publish YOLO bounding boxes."""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    def __init__(self):
        super().__init__('assignment5_yolo')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('max_fps', 4.0)
        self.declare_parameter('show_window', True)
        self.declare_parameter('log_every_n_frames', 10)

        self._topic = str(self.get_parameter('image_topic').value)
        model_path = str(self.get_parameter('model').value)
        self._confidence = float(self.get_parameter('confidence').value)
        self._max_fps = float(self.get_parameter('max_fps').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._log_every_n_frames = int(self.get_parameter('log_every_n_frames').value)
        if not (0.0 < self._confidence <= 1.0):
            raise ValueError('confidence must be in (0, 1]')
        if self._max_fps <= 0 or self._log_every_n_frames < 1:
            raise ValueError('max_fps must be positive and log_every_n_frames >= 1')

        try:
            import cv2
            from cv_bridge import CvBridge
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'Missing YOLO/OpenCV dependency; see README: '
                'ros-humble-cv-bridge, python3-opencv and pip ultralytics') from exc
        self._cv2 = cv2
        self._bridge = CvBridge()
        try:
            self._model = YOLO(model_path)
        except Exception as exc:
            raise RuntimeError(
                f'Could not load YOLO model {model_path!r}. For offline demos, '
                'set model:=/path/to/an/already-downloaded/model.pt') from exc

        self._last_inference = 0.0
        self._frame_count = 0
        self._frames_seen = 0
        self._image_pub = self.create_publisher(Image, 'yolo/annotated_image', 10)
        self._detections_pub = self.create_publisher(String, 'yolo/detections', 10)
        self.create_subscription(Image, self._topic,
                                 self._process_image, qos_profile_sensor_data)
        if self._show_window and not os.environ.get('DISPLAY'):
            self.get_logger().warning(
                'DISPLAY is missing. Disabling OpenCV window; '
                'use /yolo/annotated_image in RViz instead.')
            self._show_window = False
        self.get_logger().info(
            f'YOLO ready: topic={self._topic}, model={model_path}, '
            f'max_fps={self._max_fps:.1f}')

    def _process_image(self, message):
        self._frames_seen += 1
        now = time.monotonic()
        if now - self._last_inference < (1.0 / self._max_fps):
            return
        self._last_inference = now
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            results = self._model.predict(
                source=frame, conf=self._confidence, verbose=False)
            if not results:
                return
            result = results[0]
            annotated = result.plot()
            image_out = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image_out.header = message.header
            self._image_pub.publish(image_out)

            items = []
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                xyxy = [float(v) for v in box.xyxy[0].tolist()]
                items.append({
                    'class': str(result.names[cls_id]),
                    'confidence': round(confidence, 3),
                    'bbox_xyxy': [round(v, 1) for v in xyxy],
                })
            output = String()
            output.data = json.dumps(items, ensure_ascii=False)
            self._detections_pub.publish(output)
            self._frame_count += 1
            if self._frame_count % self._log_every_n_frames == 0:
                if items:
                    summary = ', '.join(
                        f"{item['class']} ({item['confidence']:.2f})" for item in items)
                    self.get_logger().info(f'Detected {len(items)} object(s): {summary}')
                else:
                    self.get_logger().info('No objects detected in this frame.')

            if self._show_window:
                self._cv2.imshow('TurtleBot 4 - YOLO detections', annotated)
                self._cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference or image conversion failed: {exc}')

    def destroy_node(self):
        if self._show_window:
            try:
                self._cv2.destroyAllWindows()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (RuntimeError, ValueError) as exc:
        if node is None:
            print(f'ERROR: {exc}', flush=True)
        else:
            node.get_logger().error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
