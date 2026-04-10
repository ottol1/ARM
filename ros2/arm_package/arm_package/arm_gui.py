# Team: Articulated Removable Manipulator (ARM), 2025-2026

import threading
import customtkinter as ctk
import math
#from Run import MoveARM		#run button file
#from camera_window import CameraWindow
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

class ArmGUI(Node, ctk.CTk):
    def __init__(self):
        super().__init__('arm_command_node')
        self.slider_joints = [0.0] * 5
        self.slider_lock = threading.Lock()
        self.posCommand=[180.0, 180.0, 180.0, 180.0, 180.0, 0.0]
        self.velCommand=[10.0]*6
        self.posActual = [0.0]*6
        self.velActual = [0.0]*6

 
#	publish to the arm_controller/controller_state topic
 
        self.arm_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            10
        )
 
#	publish to the gripper_controller/controller_state topic
 
        self.gripper_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            'gripper_controller/controller_state',
            10
        )
 
#	subscribe to the joint_state topic
 
        self.state_subscriber = self.create_subscription(
            JointState,
            '/joint_state',
            self.arm_state_subscriber,
            10
        )
 
 
#	publish the noint values
    # def arm_command_publisher(self):
    #     arm_command = JointTrajectoryControllerState()
    #     gripper_command = JointTrajectoryControllerState()
 
    #     # arm_command.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    #     # gripper_command.name = 'joint6'
 
    #     arm_command.desired.positions = self.posCommand[:5]
    #     arm_command.desired.velocities = self.velCommand[:5]
    #     gripper_command.desired.positions = [self.posCommand[5]]
    #     gripper_command.desired.velocities = [self.velCommand[5]]   

    def arm_command_publisher(self):
        command = JointTrajectoryControllerState()

        # REQUIRED: joint names
        command.joint_names = ['joint1','joint2','joint3','joint4','joint5']

        # REQUIRED: desired, actual, error must be JointTrajectoryPoint
        desired = JointTrajectoryPoint()
        desired.positions = [float(x) for x in self.posCommand[:5]]
        desired.velocities = [float(x) for x in self.velCommand[:5]]
        desired.accelerations = []
        desired.effort = []
        desired.time_from_start = Duration(sec=0, nanosec=0)

        actual = JointTrajectoryPoint()
        actual.positions = [float(x) for x in self.posActual[:5]]
        actual.velocities = [float(x) for x in self.velActual[:5]]
        actual.accelerations = []
        actual.effort = []
        actual.time_from_start = Duration(sec=0, nanosec=0)

        error = JointTrajectoryPoint()
        error.positions = [0.0]*5
        error.velocities = [0.0]*5
        error.accelerations = []
        error.effort = []
        error.time_from_start = Duration(sec=0, nanosec=0)

        command.desired = desired
        command.actual = actual
        command.error = error

        self.arm_publisher.publish(command)
    
    def arm_state_subscriber(self, msg):
        self.posActual = list(msg.position)
        self.velActual = list(msg.velocity)
 
 
    def set_slider_joints(self, degrees: list):
        with self.slider_lock:
            self.slider_joints = degrees[:]
            self.slider_active = True
        print(f'[Slider] { {f"J{i+1}": d for i, d in enumerate(degrees)} }')
 
    # def publish_mode(self, mode: int):
    #     print(f'[Publish] /arm/mode → {mode}')
 
    # def publish_object(self, name: str):
    #     self.publish_mode(1)
    #     print(f'[Publish] /arm/object_target → "{name}"')
 
    # def publish_joint_target(self, degrees: list):
    #     self.publish_mode(2)
    #     rads = [round(math.radians(d), 4) for d in degrees]
    #     print(f'[Publish] /arm/joint_target → {rads} rad  ({degrees} deg)')
 
    # def publish_pose_target(self, coords: list):
    #     self.publish_mode(3)
    #     print(f'[Publish] /arm/pose_target → X={coords[0]} Y={coords[1]} Z={coords[2]}')
 
    # def publish_gripper(self, state: str):
    #     print(f'[Publish] /arm/gripper → "{state}"')
 
    def destroy_node(self):
        pass
    # --------------------------
    
    # --------------------------
    # main frame
    
    def main(self):
    
        ctk.set_appearance_mode('dark')
        ctk.set_default_color_theme('blue')
    
        self.app = ctk.CTk()
        self.app.lift()
        self.app.focus_force()
        self.app.geometry('1000x680')
        self.app.title('ARM user control')
        #app.eval('tk::PlaceWindow . center')
        self.app.resizable(True, True)
    
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=0)
        self.app.grid_columnconfigure(0, weight=0, minsize=300)
        self.app.grid_columnconfigure(1, weight=1)
    
        
        camera_window = None
    
        # --------------------------
        # left frame
    
    
        left_frame = ctk.CTkFrame(self.app,width=280)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(12, 6), pady=(12,6))
        left_frame.grid_columnconfigure(0, weight=1)
        left_frame.grid_propagate(False)
        left_frame.grid_rowconfigure(4, weight=1)
        ctk.CTkLabel(left_frame, text='ARM Mode Selection',
                    font=('Arial', 14, 'bold')).pack(pady=(14, 6))
    
        text_1 = ctk.CTkTextbox(left_frame, width=240, height=70)
        text_1.pack(pady=6, padx=10)
        text_1.insert("0.0",
            "mode 1: Object detection\n"
            "mode 2: Joint variables\n"
            "mode 3: Coordinates\n"
        )
        text_1.configure(state="disabled")
    
        mode = ctk.IntVar(value=1)
    
    
        radframe = ctk.CTkFrame(left_frame)
        radframe.grid(row=2, column=0, padx=14, pady=6, sticky='ew')
    
        inframe = ctk.CTkFrame(left_frame)
        inframe.grid(row=4, column=0, sticky='nsew', padx=14, pady=10)
        inframe.grid_columnconfigure(0, weight=1)
    
        add_frame =ctk.CTkFrame(left_frame)
        add_frame.grid(row=3, column=0, padx=14, pady=(10, 6), sticky='ew')
    
        slider_frame = ctk.CTkFrame(inframe, fg_color='transparent')
        slider_frame.grid(row=0, column=0, padx=4, pady=4)
    
    
        # mode inputs
        entry_1 = ctk.CTkEntry(inframe, placeholder_text="Object", width=260)
        entry_1.grid(row=1, column=0, padx=60, pady=5)
        entry_1.grid_remove()
    
        joint_entries = []
        for i in range(5):
            j = ctk.CTkEntry(inframe, placeholder_text=f"Joint {i+1}")
            j.grid(row=i, column=0, padx=60, pady=5)
            j.grid_remove()
            joint_entries.append(j)
    
        coordinate_entries = []
        for i, lbl in enumerate(("X", "Y", "Z")):
            c = ctk.CTkEntry(inframe, placeholder_text=lbl)
            c.grid(row=i, column=0, padx=60, pady=5)
            c.grid_remove()
            coordinate_entries.append(c)
    
        JOINT_LIMITS = [(-180, 180)] * 5   # TODO: set real per-joint limits
        self.joint_sliders = []
        slider_value_labels = []
        for i, (low, high) in enumerate(JOINT_LIMITS):
            s = ctk.CTkFrame(slider_frame, fg_color='transparent')
            s.pack(fill='x', padx=14, pady=3)
    
            slider = ctk.CTkSlider(s, from_=low, to=high,orientation='horizontal', command=lambda val, idx=i: _on_slider(val, idx),)
            slider.set(0)
            slider.pack(side='left', fill='x', expand=True, padx=4)
    
            ctk.CTkLabel(s, text=f'J{i+1}', width=28, anchor='w')
    
            ctk.CTkLabel(s, text=f'{low}°', font=('Arial', 10), text_color='gray', width=36, anchor='e'
            ).pack(side='left', padx=(4, 2))
    
            ctk.CTkLabel(s, text=f'{high}°', font=('Arial', 10), text_color='gray', width=36, anchor='w'
            ).pack(side='left', padx=(2, 4))
    
            values = ctk.CTkLabel(s, text='0°', width=48, anchor='w')
            values.pack(side='left')
    
            slider_value_labels.append(values)
            self.joint_sliders.append(slider)
        slider_frame.grid_remove()
    
    
        def _on_slider(val, idx):
            slider_value_labels[idx].configure(text=f'{round(val, 1)}°')
            self.set_slider_joints([round(s.get(), 1) for s in self.joint_sliders])
    
        def reset_sliders():
            for i, s in enumerate(self.joint_sliders):
                s.set(0)
                slider_value_labels[i].configure(text='0°')
            self.set_slider_joints([0.0] * 5)
            status_label.configure(text='Sliders reset to 0°')
        # --------------------------
        # right frame
    
        right_frame = ctk.CTkFrame(self.app)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=(6,12), pady=(12,6))
    
        right_frame.grid_columnconfigure(0, weight=1)
        right_frame.grid_rowconfigure(0, weight=1)   
        right_frame.grid_rowconfigure(1, weight=0)  
        right_frame.grid_rowconfigure(2, weight=0)
        right_frame.grid_rowconfigure(3, weight=0)
    
    
        # --------------------------
        # functions
    
        def show_inputs():
            selected = mode.get()
            entry_1.grid_remove()
            slider_frame.grid_remove()
    
            for j in joint_entries:
                j.grid_remove()
            for c in coordinate_entries:
                c.grid_remove()
    
            if selected == 1:
                entry_1.grid()
            elif selected == 2:
                for j in joint_entries:
                    j.grid()
            elif selected == 3:
                for c in coordinate_entries:
                    c.grid()
            elif selected == 4:
                slider_frame.grid()

    
    
        # --------------------------
        status_label = ctk.CTkLabel(right_frame, text='', text_color='gray')
        status_label.grid(row=3, column=0, pady=(0, 6))
    
        # --------------------------
        button_frame = ctk.CTkFrame(right_frame, fg_color='transparent')
        button_frame.grid(row=1, column=0, columnspan=2, sticky='ew', padx=12, pady=(0, 10))
        button_frame.grid_columnconfigure(tuple(range(6)), weight=1)

    
        # --------------------------
        def validate(values):
            try:
                return all(v != '' and float(v) for v in values)
            except ValueError:
                return False
    
        def save_file(selected, data: dict):
            try:
                with open('user_input.txt', 'w') as f:
                    f.write(f'Mode: {selected}\n')
                    if selected == 1:
                        f.write(f"Object: {data['object']}\n")
                    elif selected == 2:
                        f.write('Joints: ' + ', '.join(str(j) for j in data['joints']) + '\n')
                    elif selected == 3:
                        f.write('Coordinates: ' + ', '.join(str(c) for c in data['coordinates']) + '\n')
                status_label.configure(text='Saved to user_input.txt', text_color='green')
            except IOError as e:
                status_label.configure(text=f'File error: {e}', text_color='red')
    
        # --------------------------
        # radio buttons
    
        ctk.CTkRadioButton(radframe, text="Mode 1",
                        variable=mode, value=1,
                        command=show_inputs).pack(anchor ='w', padx=10, pady=6)
    
        ctk.CTkRadioButton(radframe, text="Mode 2",
                        variable=mode, value=2,
                        command=show_inputs).pack(anchor ='w', padx=10, pady=6)
    
        ctk.CTkRadioButton(radframe, text="Mode 3",
                        variable=mode, value=3,
                        command=show_inputs).pack(anchor ='w', padx=10, pady=6)
    
        ctk.CTkRadioButton(radframe, text = 'Manual control',
                        variable=mode, value=4,
                        command=show_inputs).pack(anchor ='w', padx=10, pady=6)
    
    
        tabview = ctk.CTkTabview(right_frame)
        tabview.grid(row=0, column=0, sticky='nsew', padx=8, pady=(8, 4))
    
        tabview.add("joint positions")
        tabview.add("coordinates")
        tabview.add('slider values')
    
        
        joint_frame = ctk.CTkScrollableFrame(tabview.tab("joint positions"), height=80)
        joint_frame.pack(fill="both", expand=True)
        coordinate_frame = ctk.CTkScrollableFrame(tabview.tab("coordinates"), height=80)
        coordinate_frame.pack(fill="both", expand=True)
    
    
        graph = ctk.CTkFrame(right_frame)
        graph.grid(row = 2, column = 0, sticky = 'nsew', padx = 8, pady = (4,4))
        graph.grid_rowconfigure(1, weight = 1)
        graph.grid_columnconfigure(0, weight = 1)
        ctk.CTkLabel(graph, text = 'Animation').grid(row = 0, column = 0, pady = (6,2))
    
    
        # --------------------------
        joint_count = [0]
        coordinate_count = [0]
    
        def add_values():
            selected = mode.get()
            
            if selected == 2:
                vals = [j.get() for j in joint_entries]
                if not validate(vals):
                    status_label.configure(text='Invalid or missing joint values.', text_color='red')
                    return
    
                joint_count[0] += 1
    
                ctk.CTkLabel(joint_frame, text=f"{joint_count}: J1 = {vals[0]}, J2 = {vals[1]}, J3 = {vals[2]}, J4 = {vals[3]}, J5 = {vals[4]}").pack(anchor="w", pady=2)
    
                status_label.configure(text=f"Added J{joint_count[0]}: {', '.join(vals)}",text_color='green')
                for j in joint_entries: j.delete(0, 'end')
            
            elif selected == 3:
                vals = [c.get() for c in coordinate_entries]
                if not validate(vals):
                    status_label.configure(text='Invalid or missing coordinate values.', text_color='red')
                    return
    
                coordinate_count[0] += 1
    
                ctk.CTkLabel(coordinate_frame, text=f"{coordinate_count}: X = {vals[0]}, Y = {vals[1]}, Z = {vals[2]}").pack(anchor="w",pady=2)
    
                status_label.configure(
                    text=f"Added point {coordinate_count[0]}: X={vals[0]} Y={vals[1]} Z={vals[2]}",
                    text_color='green')
                for c in coordinate_entries: c.delete(0, 'end')
    
            elif selected == 4:
                vals = [slider.get for slider in self.joint_sliders]
                if not validate(vals):
                    status_label.configure(text='Invalid or missing values.', text_color='red')
                    return
                ctk.CTkLabel(joint_frame, text=f"{joint_count}: J1 = {vals[0]}, J2 = {vals[1]}, J3 = {vals[2]}, J4 = {vals[3]}, J5 = {vals[4]}").pack(anchor="w", pady=2)
    
    
    
            else:
                status_label.configure(text='Select mode 2, 3, or Manual to add values.', text_color='red')
    
        ctk.CTkButton(add_frame, text='Add Values', command=add_values).pack(pady=6, padx=10, fill='x')
    
        # --------------------------
        # run
    
        def run():
            selected = mode.get()
    
            if selected == 1:
                obj = entry_1.get()
                if not obj:
                    status_label.configure(text='No object entered', text_color='red')
                    return
                # node.publish_object(obj)
                status_label.configure(text=f'Target: {obj}', text_color='green')
        #  elif selected ==  2:
    
        #   elif selected ==  3:
    
            elif selected ==  4:
                for i in range (5):
                    self.posCommand[i] = float(self.joint_sliders[i].get())
                self.arm_command_publisher()
    
    
        # --------------------------
        def open_gripper():
            #node.publish_gripper('open')
            self.posCommand[5] = 0.0
            status_label.configure(text='Gripper opening', text_color='green')
    
        def close_gripper():
            #node.publish_gripper('close')
            self.posCommand[5] = 1.0
            status_label.configure(text='Gripper closing', text_color='green')
    
        def close():
            #nonlocal camera_window
            if camera_window is not None and camera_window.winfo_exists():
                camera_window.close()
            super().destroy_node()
            self.app.destroy()
        
        def open_camera():
            #nonlocal camera_window
            if camera_window is None or not camera_window.winfo_exists():
                camera_window = CameraWindow(self.app)
            else:
                camera_window.focus()
    
            
        # --------------------------
        for col, (txt, cmd, kw) in enumerate([
            ('Run',           run,          {'fg_color': '#2d6a4f', 'hover_color': '#1b4332'}),
            ('Open Gripper',  open_gripper, {}),
            ('Close Gripper', close_gripper,{}),
            ('Reset Sliders', reset_sliders,{'fg_color': 'gray40', 'hover_color': 'gray25'}),
            ('Camera',        open_camera,  {}),
            ('Exit',          close,        {'fg_color': '#6b1f1f', 'hover_color': '#4a1515'}),
        ]):
            ctk.CTkButton(button_frame, text=txt, command=cmd, width=100, **kw).grid(
                row=0, column=col, padx=4
            )
    
        return self.app
    
# --------------------------

    def run_ui(self):
        self.app = self.main()
        ros_thread = threading.Thread(target=rclpy.spin, daemon=True)
        ros_thread.start
        self.app.mainloop()

def main():
    rclpy.init()
    node = ArmGUI()
    try:
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()

if __name__ == '__main__':
    main()