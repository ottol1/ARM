import cv2
import sys
import os
import time
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


W = 640 # width of the image
H = 480 # height of the image

config = rs.config()
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

#model_directory = os.environ['HOME'] + '/yolov8_rs/yolov8m.pt' # Use YOLO v8m

model_directory = os.environ['HOME'] + '/yolov8s/yolov8s-world.pt' # Use YOLO World
model = YOLO(model_directory) # Always include this line for any YOLO model
model.set_classes(["person"]) # Only include this line if using YOLO World. Select what to put boxes around

prevdist = 0 # initialize "previous distance"

while True:
    #time1 = time.time()
    frames = pipeline.wait_for_frames()

    aligned_frames = align.process(frames) # I don't really understand what this does
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

    results = model(color_image) # analyze the color image

    for r in results:
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
            c = box.cls # somehow related to labeling the depth map boxes
            #width, height = depth_frame.get_width(), depth_frame.get_height()
            width, height = int(b[2])-int(b[0]), int(b[3])-int(b[1]) # attempt to calculate the size of a box
            dist = depth_frame.get_distance(width // 2, height // 2) # attempt to calculate the center of a box

            if (dist < 1.2) & (dist > 0.064): # I tested the camera's range, and this is where it was most accurate
                dist2 = f'{dist:.2f}' # convert the distance float into a string with 2 decimal places
                prevdist = dist # the current distance value becomes the previous distance value
            else:
                dist2 = f'{prevdist:.2f}' # continue showing previous distance value
            #cv2.rectangle(depth_colormap, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255), # if you want to see the depth map boxes
            #              thickness = 2, lineType=cv2.LINE_4)
            #cv2.putText(depth_colormap, text = model.names[int(c)], org=(int(b[0]), int(b[1])), # put the labels on the depth map boxes
            #            fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
            #            thickness = 2, lineType=cv2.LINE_4)
            cv2.putText(color_image, text = dist2, org=(int(b[0]), int(b[1])), # label the box with the dist2 value
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.7, color = (0, 0, 255),
                        thickness = 2, lineType=cv2.LINE_4)
            cv2.rectangle(color_image, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 0, 255), # create bounding box
                          thickness = 2, lineType=cv2.LINE_4)
    #annotated_frame = results[0].plot() # to show everything that YOLO detects, with boxes, labels, and confidence scores

    #cv2.imshow("annotated_image", annotated_frame)
    cv2.imshow("boxes_image", color_image)
    #time2 = time.time()
    #print(f"FPS : {int(1/(time2-time1))}") # to show frames per second

    if cv2.waitKey(1) & 0xFF == ord('q'): # press q to quit the process
         break