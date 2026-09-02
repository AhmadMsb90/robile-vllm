import re

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

import torch
from PIL import Image as PILImage

from transformers import (
    Qwen2_5_VLForConditionalGeneration,
    AutoProcessor,
    BitsAndBytesConfig,
)

from qwen_vl_utils import process_vision_info

from robile_interfaces.msg import (
    ObjectDetectionArray,
    VlmTarget,
)


MODEL_PATH = "/host_shared/models/Qwen2.5-VL-3B-Instruct"


class VLMNode(Node):

    def __init__(self):
        super().__init__('vlm_node')

        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.image_buffer = {}
        self.max_buffer_size = 30

        self.latest_detection_msg = None

        # ---------------------------------------------------------
        # Load Qwen2.5-VL ONCE
        # ---------------------------------------------------------

        self.get_logger().info(
            f"Loading Qwen2.5-VL-3B-Instruct from: {MODEL_PATH}"
        )

        self.processor = AutoProcessor.from_pretrained(
            MODEL_PATH
        )

        quantization_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
            bnb_4bit_use_double_quant=True,
        )

        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            MODEL_PATH,
            quantization_config=quantization_config,
            device_map="auto",
        )

        self.model.eval()

        self.get_logger().info(
            "Qwen2.5-VL model loaded successfully"
        )

        self.get_logger().info(
            f"CUDA available: {torch.cuda.is_available()}"
        )

        if torch.cuda.is_available():
            self.get_logger().info(
                f"GPU: {torch.cuda.get_device_name(0)}"
            )

        # ---------------------------------------------------------
        # ROS subscriptions
        # ---------------------------------------------------------

        self.image_subscription = self.create_subscription(
            Image,
            '/camera_front/image_raw',
            self.image_callback,
            10
        )

        self.detection_subscription = self.create_subscription(
            ObjectDetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

        self.instruction_subscription = self.create_subscription(
            String,
            '/vlm/instruction',
            self.instruction_callback,
            10
        )

        # ---------------------------------------------------------
        # ROS publisher
        # ---------------------------------------------------------

        self.target_publisher = self.create_publisher(
            VlmTarget,
            '/vlm/target',
            10
        )

        self.get_logger().info(
            "Subscribed to /camera_front/image_raw"
        )

        self.get_logger().info(
            "Subscribed to /yolo/detections"
        )

        self.get_logger().info(
            "Subscribed to /vlm/instruction"
        )

        self.get_logger().info(
            "Publishing target selection on /vlm/target"
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "VLM READY"
        )

        self.get_logger().info(
            "Waiting for RGB frames and instructions..."
        )

    # -------------------------------------------------------------
    # RGB callback
    # -------------------------------------------------------------
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='rgb8'
        )

        timestamp = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        )

        self.image_buffer[timestamp] = image

        while len(self.image_buffer) > self.max_buffer_size:
            oldest_timestamp = next(iter(self.image_buffer))
            del self.image_buffer[oldest_timestamp]

    # -------------------------------------------------------------
    # YOLO callback
    # -------------------------------------------------------------

    def detection_callback(self, msg):

        self.latest_detection_msg = msg

        self.get_logger().info(
            "Received YOLO detections: "
            + str([
                detection.class_name
                for detection in msg.detections
            ])
        )

    # -------------------------------------------------------------
    # Find RGB image matching YOLO timestamp
    # -------------------------------------------------------------

    def get_matching_image(self, detection_msg):

        timestamp = (
            detection_msg.header.stamp.sec,
            detection_msg.header.stamp.nanosec
        )

        # Exact match
        if timestamp in self.image_buffer:
            return self.image_buffer[timestamp]

        # Nearest frame fallback
        if not self.image_buffer:
            return None

        target_ns = (
            detection_msg.header.stamp.sec * 1_000_000_000
            + detection_msg.header.stamp.nanosec
        )

        closest_timestamp = min(
            self.image_buffer.keys(),
            key=lambda t: abs(
                (
                    t[0] * 1_000_000_000
                    + t[1]
                ) - target_ns
            )
        )

        return self.image_buffer[closest_timestamp]

    # -------------------------------------------------------------
    # Instruction callback
    # -------------------------------------------------------------


    def instruction_callback(self, msg):

        instruction = msg.data.strip()

        self.get_logger().info(
            f'Received instruction: "{instruction}"'
        )

        if self.latest_detection_msg is None:

            self.get_logger().warning(
                "No YOLO detections available yet."
            )

            return

        if len(self.latest_detection_msg.detections) == 0:

            self.get_logger().warning(
                "Latest YOLO message contains no detections."
            )

            return

        image = self.get_matching_image(
            self.latest_detection_msg
        )

        if image is None:

            self.get_logger().warning(
                "No RGB frame matching the YOLO detection timestamp."
            )

            return

        self.get_logger().info(
            "Starting VLM reasoning..."
        )

        response = self.run_vlm_reasoning(
            instruction,
            image,
            self.latest_detection_msg
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "VLM RESPONSE:"
        )

        self.get_logger().info(response)

        self.get_logger().info(
            "========================================"
        )

        target_id = self.parse_target_id(response)

        if target_id is None:

            self.get_logger().warning(
                "Could not parse a valid TARGET_ID from VLM response."
            )

            return

        detections = self.latest_detection_msg.detections

        if target_id < 0 or target_id >= len(detections):

            self.get_logger().warning(
                f"VLM returned invalid TARGET_ID={target_id}. "
                f"Available detections: 0-{len(detections) - 1}"
            )

            return

        target_detection = detections[target_id]

        target_msg = VlmTarget()

        target_msg.header = self.latest_detection_msg.header

        target_msg.target_id = target_id

        target_msg.target_class = (
            target_detection.class_name
        )

        target_msg.confidence = (
            target_detection.confidence
        )

        target_msg.x1 = target_detection.x1
        target_msg.y1 = target_detection.y1
        target_msg.x2 = target_detection.x2
        target_msg.y2 = target_detection.y2

        target_msg.reason = response

        self.target_publisher.publish(target_msg)

        self.get_logger().info(
            f"Selected target #{target_id}: "
            f"{target_msg.target_class} "
            f"confidence={target_msg.confidence:.2f} "
            f"bbox=("
            f"{target_msg.x1:.0f},"
            f"{target_msg.y1:.0f},"
            f"{target_msg.x2:.0f},"
            f"{target_msg.y2:.0f})"
        )

        self.get_logger().info(
            "Published selected target on /vlm/target"
        )

    # -------------------------------------------------------------
    # Parse TARGET_ID from VLM response
    # -------------------------------------------------------------

    def parse_target_id(self, response):

        match = re.search(
            r'TARGET_ID\s*:\s*(\d+)',
            response,
            re.IGNORECASE
        )

        if match is None:
            return None

        return int(match.group(1))

    # -------------------------------------------------------------
    # VLM reasoning
    # -------------------------------------------------------------

    def run_vlm_reasoning(
        self,
        instruction,
        image,
        detection_msg
    ):

        detection_text = "\nYOLO detections for this frame:\n"

        for i, detection in enumerate(
            detection_msg.detections
        ):

            detection_text += (
                f"{i}. "
                f"class={detection.class_name}, "
                f"confidence={detection.confidence:.2f}, "
                f"bbox=("
                f"{detection.x1:.0f},"
                f"{detection.y1:.0f},"
                f"{detection.x2:.0f},"
                f"{detection.y2:.0f})\n"
            )

        prompt = f"""
You are the perception and reasoning module of a mobile robot
operating in an indoor cafe environment.

You are given:
1. An RGB camera image.
2. YOLO object detections from the same image.
3. A natural-language navigation instruction.

Instruction:
"{instruction}"

{detection_text}

Select the single YOLO detection that best matches the instruction.

Use the RGB image to reason about spatial relationships such as:
- near a table
- left or right
- in front of or behind
- closest or farthest

IMPORTANT:
The detection IDs are zero-based and correspond exactly to the
numbering in the YOLO detection list.

Return EXACTLY this format:

TARGET_ID: <integer>
REASON: <one short sentence>

Do not invent a new bounding box.
Do not return coordinates.
Do not return multiple target IDs.
"""

        pil_image = PILImage.fromarray(image)

        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "image": pil_image,
                    },
                    {
                        "type": "text",
                        "text": prompt,
                    },
                ],
            }
        ]

        text = self.processor.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )

        image_inputs, video_inputs = process_vision_info(
            messages
        )

        inputs = self.processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt",
        )

        inputs = inputs.to(self.model.device)

        with torch.inference_mode():

            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=32,
                do_sample=False,
            )

        generated_ids_trimmed = [
            out_ids[len(in_ids):]
            for in_ids, out_ids in zip(
                inputs.input_ids,
                generated_ids
            )
        ]

        output_text = self.processor.batch_decode(
            generated_ids_trimmed,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False
        )

        return output_text[0].strip()


def main(args=None):

    rclpy.init(args=args)

    node = VLMNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    
