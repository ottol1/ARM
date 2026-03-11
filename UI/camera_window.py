import customtkinter
import cv2
from PIL import Image 
#----------------------------
#	create new window
class CameraWindow(customtkinter.CTkToplevel):
	def __init__(self):
		super().__init__()

		self.title("Camera Preview")
		self.geometry("640x500")

		self.cap = cv2.VideoCapture(0)
		self._running = True 

		self.status = customtkinter.CTKLabel(self, text="Starting camera ")		
		self.status.pack(fill="both", expand=True)

		self.label = customtkinter.CTkLabel(self, text="")
		self.label.pack(fill="both", expand=True)

		self.status.configure(text="Camera running")	
		self.update_frame()

#----------------------------
#	live feeed
	def streaming(self):
		if not self.running:
			return

		ret, img = self.cap.read()
		if ret:
			rgb_frame = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2RGB)
			img = Image.fromarray(rgb_frame)
			imgctk = customtkinter.CTKImage(image=img, size=(620,420))

			self.label.configure(image=imgctk)
			self.label.imgtk = imgctk

		self.after(33, self.streaming)
#----------------------------
#	close new window 
	def close(self):
		self.running = False
		if self.cap.isOpened()
			self.cap.release()
		self.destroy()
