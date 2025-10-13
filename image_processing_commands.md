#python commands:

#image resizing
cv2.resize(): Resizes the image to new dimensions.
cv2.INTER_CUBIC: Provides high-quality enlargement.
cv2.INTER_AREA: Works best for downscaling

#image rotation 
cv2.getRotationMatrix2D() : generates the transformation matrix.
cv2.warpAffine() : applies the rotation.
A positive angle rotates the image clockwise; a negative angle rotates it counterclockwise.
The scale factor adjusts the image size.

#image translation 
cv2.warpAffine() shifts the image based on translation values.
tx, ty define the movement along the x and y axes.

#image shearing 
shear_x, shear_y control the degree of skewing.
cv2.warpAffine() applies the transformation.

#image normalization 
cv2.normalize(): Normalizes pixel values.
cv2.NORM_MINMAX: Scales values between 0 and 1.
cv2.merge(): Combines separately normalized RGB channels

#edge detection 
cv2.GaussianBlur(): Removes noise through Gaussian smoothing.
cv2.Sobel(): Computes the gradient of the image.
cv2.Canny(): Applies non-maximum suppression and hysteresis thresholding to detect edges.

#image blurring 
cv2.GaussianBlur(): Smooths using a Gaussian kernel.
cv2.medianBlur(): Replaces pixels with the median value in a neighborhood..
cv2.bilateralFilter(): Preserves edges while blurring.

#morphological image processing 
cv2.dilate(): Expands object boundaries.
cv2.erode(): Shrinks object boundaries.
cv2.morphologyEx() with cv2.MORPH_OPEN: Removes small noise.
cv2.morphologyEx() with cv2.MORPH_CLOSE: Fills small holes.
