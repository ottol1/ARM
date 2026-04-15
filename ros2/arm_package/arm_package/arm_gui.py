# Team: Articulated Removable Manipulator (ARM), 2025-2026

# UI
import threading
import customtkinter as ctk
import math


# ROS
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
        self.posCommand=[0.0]*6
        self.velCommand=[10.0]*6
        self.posActual = [0.0]*6
        self.velActual = [0.0]*6
        self.switch = False
        self.posCommand_list = [0.0]*5

 
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
  

    def arm_command_publisher(self):
        command = JointTrajectoryControllerState()

        if self.switch == True:
            for i in range(6):
                self.velCommand[i] = float(self.vel_sliders[0].get())
        elif self.switch == False:
            self.velCommand=[10.0]*6

        # joint names
        command.joint_names = ['joint1','joint2','joint3','joint4','joint5']

        # desired, actual, error must be JointTrajectoryPoint
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


        grip_command = JointTrajectoryControllerState()

        # joint names
        grip_command.joint_names = ['gripper']

        # desired, actual, error must be JointTrajectoryPoint
        grip_desired = JointTrajectoryPoint()
        grip_desired.positions = [float(self.posCommand[5])]
        grip_desired.velocities = [float(self.velCommand[5]*40)]
        grip_desired.accelerations = []
        grip_desired.effort = []
        grip_desired.time_from_start = Duration(sec=0, nanosec=0)

        grip_actual = JointTrajectoryPoint()
        grip_actual.positions = [float(self.posActual[5])]
        grip_actual.velocities = [float(self.velActual[5])]
        grip_actual.accelerations = []
        grip_actual.effort = []
        grip_actual.time_from_start = Duration(sec=0, nanosec=0)

        grip_error = JointTrajectoryPoint()
        grip_error.positions = [0.0]*5
        grip_error.velocities = [0.0]*5
        grip_error.accelerations = []
        grip_error.effort = []
        grip_error.time_from_start = Duration(sec=0, nanosec=0)

        grip_command.desired = grip_desired
        grip_command.actual = grip_actual
        grip_command.error = grip_error

        self.gripper_publisher.publish(grip_command)

        
    
    def arm_state_subscriber(self, msg):
        self.posActual = list(msg.position)
        self.velActual = list(msg.velocity)
 
 
    def set_slider_joints(self, degrees: list):
        with self.slider_lock:
            self.slider_joints = degrees[:]
            self.slider_active = True
        print(f'[Slider] { {f"J{i+1}": d for i, d in enumerate(degrees)} }')
 

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
        slider_frame.pack(fill='x', padx=4, pady=4)
        vel_frame =ctk.CTkFrame(left_frame)
        vel_frame.grid(row=5, column=0, padx=14, pady=(10, 6), sticky='ew')

        # mode inputs
        entry_1 = ctk.CTkEntry(inframe, placeholder_text="Object", width=260)
        entry_1.pack(pady=5)
        entry_1.pack_forget()
        joint_entries = []
        for i in range(5):
            j = ctk.CTkEntry(inframe, placeholder_text=f"Joint {i+1} (rad)")
            j.pack(pady=5)
            j.pack_forget()
            joint_entries.append(j)
        self.coordinate_entries = []
        for i, lbl in enumerate(("X", "Y", "Z")):
            c = ctk.CTkEntry(inframe, placeholder_text=lbl)
            c.pack(pady=5)
            c.pack_forget()
            self.coordinate_entries.append(c)
        JOINT_LIMITS = [(-math.pi/2, math.pi/2)] * 5   # TODO: set real per-joint limits
        JOINT_LIMITS[0] = (-math.pi, math.pi)
        self.joint_sliders       = []
        slider_value_labels = []
        for i, (low, high) in enumerate(JOINT_LIMITS):
            s = ctk.CTkFrame(slider_frame, fg_color='transparent')
            s.pack(fill='x', padx=14, pady=3)
            ctk.CTkLabel(s, text=f'J{i+1}', width=28, anchor='w').pack(side='left')
            ctk.CTkLabel(s, text='-pi', font=('Arial', 10), text_color='gray', width=36, anchor='e'
            ).pack(side='left', padx=(4, 2))
            ctk.CTkLabel(s, text="pi", font=('Arial', 10), text_color='gray', width=36, anchor='w'
            ).pack(side='right', padx=(2, 4))

            slider = ctk.CTkSlider(s, from_=low, to=high,orientation='horizontal', command=lambda val, idx=i: _on_slider(val, idx),)
            slider.set(0)
            slider.pack(side='left', fill='x', expand=True, padx=4)

            values = ctk.CTkLabel(s, text='0 rad', width=48, anchor='w')
            values.pack(side='left')
            slider_value_labels.append(values)
            self.joint_sliders.append(slider)
        slider_frame.pack_forget()

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
            entry_1.pack_forget()
            slider_frame.pack_forget()
    
            for j in joint_entries:
                j.pack_forget()
            for c in self.coordinate_entries:
                c.pack_forget()
    
            if selected == 1:
                entry_1.pack()
            elif selected == 2:
                for j in joint_entries:
                    j.pack()
            elif selected == 3:
                for c in self.coordinate_entries:
                    c.pack()
            elif selected == 4:
                slider_frame.pack(fill='x', padx=4, pady=4)

    
    
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

                self.posCommand_list.append([vals]) # WE NEED TO CHANGE THIS LATER, this is only so that we can do some demonstrations a bit faster

                joint_count[0] += 1
    
                ctk.CTkLabel(joint_frame, text=f"{joint_count}: J1 = {vals[0]}, J2 = {vals[1]}, J3 = {vals[2]}, J4 = {vals[3]}, J5 = {vals[4]}").pack(anchor="w", pady=2)
    
                status_label.configure(text=f"Added J{joint_count[0]}: {', '.join(vals)}",text_color='green')
                for j in joint_entries: j.delete(0, 'end')

                
            
            elif selected == 3:
                vals = [c.get() for c in self.coordinate_entries]
                if not validate(vals):
                    status_label.configure(text='Invalid or missing coordinate values.', text_color='red')
                    return
    
                coordinate_count[0] += 1
    
                ctk.CTkLabel(coordinate_frame, text=f"{coordinate_count}: X = {vals[0]}, Y = {vals[1]}, Z = {vals[2]}").pack(anchor="w",pady=2)
    
                status_label.configure(
                    text=f"Added point {coordinate_count[0]}: X={vals[0]} Y={vals[1]} Z={vals[2]}",
                    text_color='green')
                for c in self.coordinate_entries: c.delete(0, 'end')
    
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
        # velocity
        VEL_LIMITS = [(0, 100)] * 5   # TODO: set real per-joint limits
        self.vel_sliders       = []
        vel_value_labels = []
        def vel():
        
            if switch_var.get() == 'on':
                self.switch = True
                v.pack(fill='x', padx=10, pady=(0, 6))
            else:
                self.switch = False
                v.pack_forget()
    
            
        v = ctk.CTkFrame(vel_frame, fg_color='transparent')
        ctk.CTkLabel(v, text=f'Velocity', width=28, anchor='w').pack(side='top')
    
    

        ctk.CTkLabel(v, text=f'{0}', font=('Arial', 10), text_color='gray', width=36, anchor='e'
        ).pack(side='left', padx=(4, 2))
        ctk.CTkLabel(v, text=f'{100}', font=('Arial', 10), text_color='gray', width=36, anchor='w'
        ).pack(side='right', padx=(2, 4))
        vel_slider = ctk.CTkSlider(v, from_=0, to=100,orientation='horizontal', command=lambda val: vel_value.configure(text=f'{round(val, 1)}'))
        vel_slider.set(0)
        vel_slider.pack(side='left', fill='x', expand=True, padx=4)
        vel_value = ctk.CTkLabel(v, text='0', width=48, anchor='w')
        vel_value.pack(side='left')
        vel_value_labels.append(vel_value)
        self.vel_sliders.append(vel_slider)
        switch_var = ctk.StringVar(value="off")
        ctk.CTkSwitch(vel_frame, text='ARM Velocity', command=vel, variable=switch_var, onvalue="on", offvalue="off").pack(pady=6, padx=10, fill='x')


        def vect_compare(posCommand, posActual):
            for i in range(len(posCommand)):
                if (posActual[i] > (posCommand[i] + 1)) or (posActual[i] < (posCommand[i] - 1)):
                    return False
            
            return True


            # --------------------------
            # run


        def run():
            selected = mode.get()
    
            if selected == 1:
                obj = entry_1.get()
                if not obj:
                    status_label.configure(text='No object entered', text_color='red')
                    return
                status_label.configure(text=f'Target: {obj}', text_color='green')
            
            elif selected ==  2:
                for i in range(len(self.posCommand_list)):
                    while vect_compare(self.posCommand, self.posActual[i]) == False:
                        for j in range(5):
                            self.posCommand[i] = self.posCommand_list[i][j]
                        self.arm_command_publisher()
                
                #   elif selected ==  3:
                # xyz = [0.0]*3
                # for i in range(3):
                #     xyz[i] = float(self.coordinate_entries.get())
                # self.posCommand = xyz_inverse(xyz)
    
            elif selected ==  4:
                for i in range (5):
                    self.posCommand[i] = float(self.joint_sliders[i].get())
                self.arm_command_publisher()
        # --------------------------
        # def xyz_inverse(xyz):
            
        #     d1 = 57.48 # mm
        #     a2 = 140.05
        #     d2 = 2
        #     a3 = 143.19
        #     a4 = 11
        #     d5 = 151.74

        #     a = 
        #     q1 = math.atan2(xyz[1], xyz(0))
        #     q2y = 
        #     q2x = 
        #     q2 = atan2(,)
        #     return

    
        # --------------------------
        def open_gripper():
            #node.publish_gripper('open')
            self.posCommand[5] = 0.0
            status_label.configure(text='Gripper opening', text_color='green')
    
        def close_gripper():
            #node.publish_gripper('close')
            self.posCommand[5] = 1.0
            status_label.configure(text='Gripper closing', text_color='green')
    
        
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
