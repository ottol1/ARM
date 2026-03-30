import customtkinter as ctk
import configparser
import subprocess as sp
import select

from pathlib import Path


class ODDARMLaunch(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.geometry("600x400")
        self.title("ODD ARM Launchpad")
        self.grid_rowconfigure(5, weight=1)
        self.columnconfigure(1, weight=1)
        
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        
        self.odd_button = ctk.CTkButton(self, text="ODD", width=200, command=self.launch_odd)
        self.odd_button.grid(row=0, column=0, padx=20, pady=5)
        #self.odd_button.pack(padx=20, pady=20)
        
        self.arm_button = ctk.CTkButton(self, text="ARM", width=200, command=self.launch_arm)
        self.arm_button.grid(row=1, column=0, padx=20, pady=5)
        #self.arm_button.pack(padx=20, pady=20)
        
        self.cv_button = ctk.CTkButton(self, text="Computer Vision", width=200, command=self.launch_cv)
        self.cv_button.grid(row=2, column=0, padx=20, pady=5)
        
        self.log_label = ctk.CTkLabel(self, text="Console Log")
        self.log_label.grid(row=3, column=0, padx=0, pady=0)
        
        self.combined_log = ctk.CTkTextbox(master=self, width=600-40)
        self.combined_log.grid(row=4, column=0, sticky="nsew", padx=20, pady=0)
        #self.combined_log.insert("0.0", "test\n"*50)
        self.combined_log.configure(state="disabled")
        
        self.odd_process = None
        self.cv_process = None
        self.arm_process = None
        
        self.running = True
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.process_log()


    def add_to_log(self, text):
        self.combined_log.configure(state="normal")
        self.combined_log.insert("0.0", text)
        self.combined_log.configure(state="disabled")


    def process_log(self):
        self._process_log("odd")
        
        if self.running:
            self.after(10, self.process_log)
    
    
    def _process_log(self, name):
        process = getattr(self, name + "_process")
        if process is None:
            return
    
        lines_added = 0
    
        stdout_ready, _, _ = select.select([process.stdout], [], [], 0.01)
        while stdout_ready and lines_added < 100:
            line = process.stdout.readline()
            if line != b'':
                self.add_to_log(line)
                lines_added += 1
            else:
                break
            stdout_ready, _, _ = select.select([process.stdout], [], [], 0.01)
            
        stderr_ready, _, _ = select.select([process.stderr], [], [], 0.01)
        while stderr_ready and lines_added < 100:
            line = process.stdout.readline()
            if line == b'':
                self._close_process("odd")
                break
            else:
                self.add_to_log(line)
                lines_added += 1
            stderr_ready, _, _ = select.select([process.stderr], [], [], 0.01)

        if lines_added >= 100:
            self.add_to_log("Way too many prints!\n")

    def _close_process(self, name):
        process = getattr(self, name + "_process")
        if process is None:
            return
        
        try:
            process.communicate(timeout=1)
        except sp.TimeoutExpired:
            pass
            
        process.terminate()
        process.terminate()
        setattr(self, name + "_process", None)


    def _launch_process(self, name):
        if getattr(self, name + "_process") is not None:
            print("Already created!")
            return
    
        script_path = Path(self.config.get("paths", name))
        setattr(self, name + "_process", sp.Popen(script_path, stdout=sp.PIPE, stderr=sp.PIPE, cwd=script_path.parent))
    
    
    def launch_odd(self):
        self._launch_process("odd")
        
        
    def launch_arm(self):
        self._launch_process("arm")
    
    
    def launch_cv(self):
        self._launch_process("cv")
    
    
    def on_close(self):
        self.running = False
    
        self._close_process("odd")
        self._close_process("cv")
        self._close_process("arm")
        
        self.destroy()


app = ODDARMLaunch()
app.mainloop()
