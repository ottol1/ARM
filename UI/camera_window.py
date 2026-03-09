import customtkinter
import cv2

#----------------------------
#	create new window
class camerawindow(customtkinter.CTkToplevel):
	def __init__(self):
		super().__init__()

		self.title("Camera Preview")
		self.geometry("640x480")

		self.cap = cv2.VideoCapture(0)

		self.label = customtkinter.CTkLabel(self, text="")
		self.label.pack(fill="both", expand=True)

		self.update_frame()

#----------------------------
#	
	def streaming(self):
		ret, img = self.cap.read()
	
		if ret:
			frame = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2RGB)
			img = Image.fromarray(frame)
			imgtk = ImageTk.PhotoImage(image=img)

			self.label.imgtk = imgtk
			self.label.configure(image=imgtk)

		self.after(2, self.update_frame)
		print("Camera started successfully")
#----------------------------
#	close new window 
	def close(self):
		self.cap.release()
		self.destroy()
