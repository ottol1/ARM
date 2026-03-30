import rclpy
from rclpy.logging import LoggingSeverity
import customtkinter as ctk
from rclpy.node import Node
from odd_package_interfaces.msg import RPM, EncoderTicks, BumpSensors, Voltage, PID, Temperature
from time import time
from math import floor
from odd_package.odd_control import ODDNode
from geometry_msgs.msg import Vector3
import sys
from std_srvs.srv import Trigger


class MotorFrame(ctk.CTkFrame):
    def __init__(self, master, font=None, **kwargs):
        super().__init__(master, **kwargs)
        
        self.motor_label = ctk.CTkLabel(self, text="MOTORS", font=font)
        self.motor_label.grid(row=0, column=0, padx=0, pady=0)
        
        self.motor_label_left = ctk.CTkLabel(self, text="LEFT", font=font, text_color="red")
        self.motor_label_left.grid(row=1, column=0, padx=0, pady=0)
        
        self.motor_label_right = ctk.CTkLabel(self, text="RIGHT", font=font, text_color="blue")
        self.motor_label_right.grid(row=1, column=1, padx=0, pady=0)
        
        self.rpm_left_label = ctk.CTkLabel(self, text="LEFT rpm", font=font, text_color="red")
        self.rpm_left_label.grid(row=2, column=0, padx=0, pady=0)
        
        self.rpm_right_label = ctk.CTkLabel(self, text="RIGHT rpm", font=font, text_color="blue")
        self.rpm_right_label.grid(row=2, column=1, padx=0, pady=0)


class IMUFrame(ctk.CTkFrame):
    def __init__(self, master, font=None, **kwargs):
        super().__init__(master, **kwargs)
        
        self.imu_label = ctk.CTkLabel(self, text="IMU", font=font)
        self.imu_label.grid(row=0, column=1, padx=0, pady=0)
        
        self.x_label = ctk.CTkLabel(self, text="X", font=font, text_color="red")
        self.x_label.grid(row=1, column=0, padx=0, pady=0)
        
        self.y_label = ctk.CTkLabel(self, text="Y", font=font, text_color="green")
        self.y_label.grid(row=1, column=1, padx=0, pady=0)
        
        self.z_label = ctk.CTkLabel(self, text="Z", font=font, text_color="blue")
        self.z_label.grid(row=1, column=2, padx=0, pady=0)
        
        self.x_accel_label = ctk.CTkLabel(self, text="X", font=font, text_color="red")
        self.x_accel_label.grid(row=3, column=0, padx=5, pady=0)
        
        self.y_accel_label = ctk.CTkLabel(self, text="Y", font=font, text_color="green")
        self.y_accel_label.grid(row=3, column=1, padx=5, pady=0)
        
        self.z_accel_label = ctk.CTkLabel(self, text="Z", font=font, text_color="blue")
        self.z_accel_label.grid(row=3, column=2, padx=5, pady=0)
        
        self.x_rot_label = ctk.CTkLabel(self, text="X", font=font, text_color="red")
        self.x_rot_label.grid(row=2, column=0, padx=0, pady=0)
        
        self.y_rot_label = ctk.CTkLabel(self, text="Y", font=font, text_color="green")
        self.y_rot_label.grid(row=2, column=1, padx=0, pady=0)
        
        self.z_rot_label = ctk.CTkLabel(self, text="Z", font=font, text_color="blue")
        self.z_rot_label.grid(row=2, column=2, padx=0, pady=0)
        
        self.temperature_label = ctk.CTkLabel(self, text="77", font=font)
        self.temperature_label.grid(row=4, column=1, padx=0, pady=0)
    
    
    def update_orientation(self, msg):
        self.x_rot_label.configure(text = f"{msg.x: 04.0f}°")
        self.y_rot_label.configure(text = f"{msg.y: 04.0f}°")
        self.z_rot_label.configure(text = f"{msg.z: 04.0f}°")
    
    
    def update_acceleration(self, msg):
        self.x_accel_label.configure(text = f"{msg.x: 03.2f} m/s^2")
        self.y_accel_label.configure(text = f"{msg.y: 03.2f} m/s^2")
        self.z_accel_label.configure(text = f"{msg.z: 03.2f} m/s^2")


    def update_temperature(self, msg):
        self.temperature_label.configure(text = f"{msg.temperature} °C")


class BumperFrame(ctk.CTkFrame):
    def __init__(self, master, font=None, **kwargs):
        super().__init__(master, **kwargs)
    
        self.bumper_label = ctk.CTkLabel(self, text="BUMPERS", font=font)
        self.bumper_label.grid(row=0, column=1, padx=0, pady=0)
        
        self.left_label = ctk.CTkLabel(self, text="L", font=font, text_color="green")
        self.left_label.grid(row=1, column=0, padx=5, pady=0)
        
        self.front_label = ctk.CTkLabel(self, text="F", font=font, text_color="green")
        self.front_label.grid(row=1, column=1, padx=5, pady=0)
        
        self.right_label = ctk.CTkLabel(self, text="R", font=font, text_color="green")
        self.right_label.grid(row=1, column=2, padx=5, pady=0)
        
        self.back_label = ctk.CTkLabel(self, text="B", font=font, text_color="green")
        self.back_label.grid(row=2, column=1, padx=5, pady=0)
        
    
    def update_labels(self, msg):
        self.left_label.configure(text_color="red" if msg.left else "green")
        self.right_label.configure(text_color="red" if msg.right else "green")
        self.front_label.configure(text_color="red" if msg.front else "green")
        self.back_label.configure(text_color="red" if msg.back else "green")


class ODDGUI(ctk.CTk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.geometry("600x400")
        self.title("ODD Control GUI")
        self.bind("<KeyPress>", self.on_keypress)
        self.bind("<KeyRelease>", self.on_key_release)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.running = True
        self.last_update = time()
        self.rpm_publisher = self.node.create_publisher(RPM, 'rpm_command', 10)
        self.mono_font = ctk.CTkFont(family="mono", size=16)
        
        self.columnconfigure(1, weight=1)
        self.rowconfigure(4, weight=1)
        
        
        self.imu_frame = IMUFrame(self, font=self.mono_font)
        self.imu_frame.grid(row=0, column=0, padx=10, pady=10)
        
        #self.imu_label_text = ["IMU", "", ""]
        #self.imu_label = ctk.CTkLabel(self, text="IMU\n \n ", font=self.mono_font)
        #self.imu_label.grid(row=0, column=0, padx=10, pady=10)
        
        self.motor_frame = MotorFrame(self, font=self.mono_font)
        self.motor_frame.grid(row=1, column=0, padx=10, pady=10)
        
        self.rpm_label = ctk.CTkLabel(self, text="RPM\n 0, 0", font=self.mono_font)
        self.rpm_label.grid(row=1, column=0, padx=10, pady=10)
        
        self.encoder_label = ctk.CTkLabel(self, text="ENCODER TICKS\n 0, 0", font=self.mono_font)
        self.encoder_label.grid(row=2, column=0, padx=10, pady=10)
        
        self.bump_frame = BumperFrame(self, font=self.mono_font)
        self.bump_frame.grid(row=3, column=0, padx=10, pady=10)
        #self.bump_label = ctk.CTkLabel(self, text="bump", font=self.mono_font)
        #self.bump_label.grid(row=3, column=0, padx=10, pady=10)
        
        self.battery_label = ctk.CTkLabel(self, text="batt", font=self.mono_font)
        self.battery_label.grid(row=4, column=0, padx=10, pady=10)
        
        self.temperature_label = ctk.CTkLabel(self, text="temp", font=self.mono_font)
        self.temperature_label.grid(row=5, column=0, padx=10, pady=10)
        
        self.wasd_enabled = ctk.BooleanVar(self)
        self.wasd_switch = ctk.CTkCheckBox(self, text="WASD Control", command=self.wasd_mode_updated, variable=self.wasd_enabled)
        self.wasd_switch.grid(row=5, column=0, padx=10, pady=10)
        
        self.stop_button = ctk.CTkButton(self, text="Halt", command=self.stop_moving)
        self.stop_button.grid(row=6, column=0, padx=10, pady=10)
        
        self.orientation_subscription = self.node.create_subscription(Vector3, 'orientation_deg', self.imu_frame.update_orientation, 10)
        self.linear_subscription = self.node.create_subscription(Vector3, 'linear_accel', self.imu_frame.update_acceleration, 10)
        self.rpm_subscription = self.node.create_subscription(RPM, 'rpm_actual', self.set_rpm_label, 10)
        self.encoder_subscription = self.node.create_subscription(EncoderTicks, 'encoders', self.set_encoder_label, 10)
        self.bump_subscription = self.node.create_subscription(BumpSensors, 'bump_sensors', self.bump_frame.update_labels, 10)
        self.voltage_subscription = self.node.create_subscription(Voltage, 'battery_voltage', self.set_battery_label, 10)
        self.temperature_subscription = self.node.create_subscription(Temperature, 'imu_temperature', self.imu_frame.update_temperature, 10)
        
        self.shutdown_control_client = self.node.create_client(Trigger, 'shutdown_control')
        
        self.velocity = [0.0, 0.0]
        self.vel_clamp = 127.0
        self.request_velocity = [0.0, 0.0]
        #self.accel_clamp = 30.0
        self.accel = 300.0
    
        # W A S D
        self.pressed = {25: False, 38: False, 39: False, 40: False}
        
        self.key_release_id = None
    
    
    def stop_moving(self):
        self.wasd_enabled.set(False)
        rpm = RPM()
        rpm.theta_dot_left = 0.0
        rpm.theta_dot_right = 0.0
        self.velocity = [0.0, 0.0]
        self.pressed = {25: False, 38: False, 39: False, 40: False}
        self.rpm_publisher.publish(rpm)
    
    
    def wasd_mode_updated(self):
        if self.wasd_enabled.get():
            return
    
        self.stop_moving()
    
    
    def set_battery_label(self, msg):
        self.battery_label.configure(text = f"{msg.volts: 03.2f}V")
    
    
    #def set_temperature_label(self, msg):
    #    pass
    
    
    #def set_orientation_label(self, msg):
    #    self.imu_label_text[1] = f"x: {msg.x: 04.0f}°, y: {msg.y: 04.0f}°, z: {msg.z: 04.0f}°"
    #    self.imu_label.configure(text = "\n".join(self.imu_label_text))
    
    
    #def set_linear_label(self, msg):
    #    self.imu_label_text[2] = f"x: {msg.x: 03.2f} m/s^2, y: {msg.y: 03.2f} m/s^2, z: {msg.z: 03.2f} m/s^2"
    #    self.imu_label.configure(text = "\n".join(self.imu_label_text))
    
    
    def set_rpm_label(self, msg):
        self.rpm_label.configure(text = f"RPM\n{msg.theta_dot_left: 04.0f} left, {msg.theta_dot_right: 04.0f} right")
    
    
    def set_encoder_label(self, msg):
        self.encoder_label.configure(text = f"ENCODER TICKS\n{msg.left: 05d} left, {msg.right: 05d} right")
    
    
    def set_bump_label(self, msg):
        pass
        #on_off = lambda state: "X" if state else " "
        #self.bump_label.configure(text = f"BUMPERS\n{on_off(msg.left)}, {on_off(msg.front)}, {on_off(msg.right)}\n{on_off(msg.back)}")
    
    
    def update(self):
        super().update()
        delta = time() - self.last_update
        self.last_update = time()
        
        self.request_velocity[0] = 0.0
        self.request_velocity[1] = 0.0
        
        if self.pressed[25]:
            self.request_velocity[0] += 1
            self.request_velocity[1] += 1
        if self.pressed[39]:
            self.request_velocity[0] -= 1
            self.request_velocity[1] -= 1
        
        if self.pressed[38]:
            self.request_velocity[0] -= 1
            self.request_velocity[1] += 1
        if self.pressed[40]:
            self.request_velocity[0] += 1
            self.request_velocity[1] -= 1
        
        self.request_velocity[0] *= self.vel_clamp
        self.request_velocity[1] *= self.vel_clamp
        
        if self.request_velocity[0] > self.velocity[0]:
            self.velocity[0] = min(self.velocity[0] + delta*self.accel, self.request_velocity[0])
        elif self.request_velocity[0] < self.velocity[0]:
            self.velocity[0] = max(self.velocity[0] - delta*self.accel, self.request_velocity[0])
       
        
        if self.request_velocity[1] > self.velocity[1]:
            self.velocity[1] = min(self.velocity[1] + delta*self.accel, self.request_velocity[1])
        elif self.request_velocity[1] < self.velocity[1]:
            self.velocity[1] = max(self.velocity[1] - delta*self.accel, self.request_velocity[1])
        
        
        
        #self.velocity[0] += min(max((self.request_velocity[0] - self.velocity[0])*delta*self.accel, -self.accel_clamp), self.accel_clamp)
        #self.velocity[1] += min(max((self.request_velocity[1] - self.velocity[1])*delta*self.accel, -self.accel_clamp), self.accel_clamp)
        
        self.velocity[0] = max(min(float(floor(self.velocity[0])), self.vel_clamp), -self.vel_clamp)
        self.velocity[1] = max(min(float(floor(self.velocity[1])), self.vel_clamp), -self.vel_clamp)
        
        #self.node.get_logger().info(str(self.velocity))
        
        rpm = RPM()
        rpm.theta_dot_left = self.velocity[0]
        rpm.theta_dot_right = self.velocity[1]
        
        if self.wasd_enabled.get():
            self.rpm_publisher.publish(rpm)
        
        
    def on_keypress(self, event):
        self.pressed[event.keycode] = True
        if self.key_release_id is not None:
            self.after_cancel(self.key_release_id)
            #self.node.get_logger().info("a")
            self.key_release_id = None
        
    
    def on_key_release(self, event):
        self.key_release_id = self.after(50, self.key_release, event)
    
    
    def key_release(self, event):
        self.pressed[event.keycode] = False
        
    
    def on_close(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    app = ODDGUI(rclpy.create_node("odd_gui"))
    try:
        while app.running:
            app.update()
            rclpy.spin_once(app.node, timeout_sec=0.01)
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy()
        app.quit()
        app.shutdown_control_client.call_async(Trigger.Request())
        app.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
