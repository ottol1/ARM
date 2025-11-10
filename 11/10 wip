from ultralytics import YOLO
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import pyrealsense2 as rs

model = YOLO('yolov8n.pt')


pip = rs.pipeline() #look into
config = rs.config()


#FRAME SIZES
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pip.start(config)
print("Camera started successfully")

backsubtraction = cv2.createBackgroundSubtractorMOG2()

depth_sense = profile.get_device().first_depth_sensor()
depth_scales = depth_sense.get_depth_scale()

align_to = rs.stream.depth
align = rs.align(align_to)


try:
	while True:

		frames = pip.wait_for_frames()
		alignment = align.process(frames) 

		color_frame = frames.get_color_frame()
		depth_frame = frames.get_depth_frame()

		#ARRAYS
		c_frame = np.asanyarray(color_frame.get_data())
		d_frame = np.asanyarray(depth_frame.get_data())
		
		
		img_rgb = cv2.cvtColor(c_frame, cv2.COLOR_BGR2RGB)
		fg_mask = backsubtraction.apply(c_frame)	#backgroun subtaction

		

		#results = model.predict(c_frame, verbose = False)
		#for r in results: 
			#for box in r.boxes:
				#x1, x2, y1, y2 = map(int, box.xyxy[0])
				#cv2.rectangle(c_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

				#midpoint_x = int( (x1 + x2)/2 )	
				#midpoint_y = int( (y1 + y2)/2 )
				distance = depth_frame[midpoint_x, midpoint_y] * depth_scale
				
				#zdepth = depth.get_distance(midpoint_x, midpoint_y)
				
			
		contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		frame_cp = cv2.drawContours(c_frame, contours, -1, (0, 250,0), 2)

		cv2.imshow("contour frame", frame_cp)
		cv2.imshow("real feed", c_frame)
		cv2.imshow("fg mask", fg_mask)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
finally:
	# Stop streaming
    	pip.stop()
    	cv2.destroyAllWindows()

#reval, mask_threshold = cv2.threshold(fg_mask, 180, 250, cv2.TRESH_BINARY)


