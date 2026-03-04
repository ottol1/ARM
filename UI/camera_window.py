import customtkinter
import cv2
from PIL import Image, ImageTk

class camerawindow(customtkinter.CTkToplevel):
    def init(self):
        super().init()

        self.title("Camera Preview")
        self.geometry("640x480")

        self.cap = cv2.VideoCapture(0)

        self.label = customtkinter.CTkLabel(self, text="")
        self.label.pack(fill="both", expand=True)

        self.update_frame()

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(img)

            self.label.imgtk = imgtk
            self.label.configure(image=imgtk)

        self.after(10, self.update_frame)

    def close(self):
        self.cap.release()
        self.destroy()
