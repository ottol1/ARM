# SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as im
from nanoowl.tree import Tree
from nanoowl.tree_predictor import (
    TreePredictor
)
from nanoowl.tree_drawing import draw_tree_output
from nanoowl.owl_predictor import OwlPredictor

class Nano_OWL_Subscriber(Node):

    def __init__(self):
        super().__init__('nano_owl_subscriber')
        
        self.declare_parameter('model', 'google/owlvit-base-patch32')
        self.declare_parameter('image_encoder_engine', '../data/owl_image_encoder_patch32.engine')
        self.declare_parameter('thresholds', rclpy.Parameter.Type.DOUBLE)

        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)
        self.query_subscription  # prevent unused variable warning

        # Subscriber for input image
        self.image_subscription = self.create_subscription(
            Image,
            'input_image',
            self.listener_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 

        self.output_publisher = self.create_publisher(Detection2DArray, 'output_detections', 10)
        self.output_image_publisher = self.create_publisher(Image, 'output_image', 10)

        self.image_encoder_engine = self.get_parameter('image_encoder_engine').get_parameter_value().string_value

        self.predictor = TreePredictor(
            owl_predictor=OwlPredictor(
                image_encoder_engine=self.image_encoder_engine
            )
        )

        self.query = "[a person]"

    def query_listener_callback(self, msg):
        self.query = msg.data


    def listener_callback(self, data):
        input_query = self.query
        input_model = self.get_parameter('model').get_parameter_value().string_value
        input_image_encoder_engine = self.get_parameter('image_encoder_engine').get_parameter_value().string_value
        thresholds = self.get_parameter('thresholds').get_parameter_value().double_value

        # call model with input_query and input_image 
        cv_img = self.cv_br.imgmsg_to_cv2(data, 'rgb8')
        PIL_img = im.fromarray(cv_img)

        # Parsing input text prompt
        self.get_logger().info('Your query: %s' % input_query)
        tree = Tree.from_prompt(input_query)
        label_map = tree.get_label_map()
        clip_encodings = self.predictor.encode_clip_text(tree)
        owl_encodings = self.predictor.encode_owl_text(tree)
        
        # thresholds = [thresholds] * len(text)

        output = self.predictor.predict(
            PIL_img,
            tree=tree,
            clip_text_encodings=clip_encodings,
            owl_text_encodings=owl_encodings
        )

        detections_arr = Detection2DArray()
        detections_arr.header = data.header

        for raw_detection in output.detections:
            box = [float(x) for x in raw_detection.box]
            top_left = (box[0], box[1])
            bottom_right = (box[2], box[3])
            obj = Detection2D()
            obj.bbox.size_x = abs(box[2] - box[0])
            obj.bbox.size_y = abs(box[1] - box[3])
            obj.bbox.center.position.x = (box[0] + box[2]) / 2.0 
            obj.bbox.center.position.y = (box[1] + box[3]) / 2.0
            
            for i, label in enumerate(raw_detection.labels):
                label_index = int(label)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label_map[label_index]
                hyp.hypothesis.score = raw_detection.scores[i]
                obj.results.append(hyp)
            
            obj.header = data.header
            detections_arr.detections.append(obj)

        self.output_publisher.publish(detections_arr)

        image = draw_tree_output(cv_img, output, tree)
        # convert RGB to BGR
        image = image[:, :, ::-1].copy()

        self.output_image_publisher.publish(self.cv_br.cv2_to_imgmsg(image, "bgr8"))



def main(args=None):
    rclpy.init(args=args)

    nano_owl_subscriber = Nano_OWL_Subscriber()

    rclpy.spin(nano_owl_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
