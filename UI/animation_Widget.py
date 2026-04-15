# -------------------------- imports neeeded ---------------------------------
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
# ----------------------------------------------------------------------------


# -----------------------Joint State Subscriber ------------------------------
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_gui_subscriber')
        self.joint_positions = [0.0] * 5
        self.lock = threading.Lock()
        self.subscription = self.create_subscription(
            JointState, '/joint_state', self.listener_callback, 10)
        self.get_logger().info("JointStateSubscriber started")

    def listener_callback(self, msg):
        with self.lock:
            for i, name in enumerate(msg.name):
                if i < 5 and 'joint' in name.lower():
                    idx = int(''.join(filter(str.isdigit, name))) - 1
                    if 0 <= idx < 5:
                        self.joint_positions[idx] = msg.position[i]

    def get_joints(self):
        with self.lock:
            return self.joint_positions[:]
#------------------------------------------------------------------------------

# ------------------------ forward kinematics function ------------------------
def forward_kinematics(joints_deg):
    """Updated with your latest MATLAB parameters.
    Zero position = arm pointing straight up (t2 offset applied)."""
    t1, t2, t3, t4, t5 = np.deg2rad(joints_deg)
    
    # New dimensions from your latest script
    d1 = 57.48
    a2 = 140.05
    d2 = 2.0      # offset in A2
    a3 = 143.19
    a4 = 11.0
    d5 = 151.74

    # A1
    A1 = np.array([
        [np.cos(t1), 0,          np.sin(t1), 0],
        [np.sin(t1), 0,         -np.cos(t1), 0],
        [0,          1,          0,          d1],
        [0,          0,          0,          1]
    ])

    # A2 (with d2 offset)
    A2 = np.array([
        [np.cos(t2), -np.sin(t2), 0, a2*np.cos(t2)],
        [np.sin(t2),  np.cos(t2), 0, a2*np.sin(t2)],
        [0,           0,          1, d2],
        [0,           0,          0, 1]
    ])

    # A3
    A3 = np.array([
        [np.cos(t3), -np.sin(t3), 0, a3*np.cos(t3)],
        [np.sin(t3),  np.cos(t3), 0, a3*np.sin(t3)],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])

    # A4
    A4 = np.array([
        [np.cos(np.pi/2 + t4), 0,          np.sin(np.pi/2 + t4), a4*np.cos(t4)],
        [np.sin(np.pi/2 + t4), 0,         -np.cos(np.pi/2 + t4), a4*np.sin(t4)],
        [0,                    1,          0,                    0],
        [0,                    0,          0,                    1]
    ])

    # A5
    A5 = np.array([
        [np.cos(t5), -np.sin(t5), 0, 0],
        [np.sin(t5),  np.cos(t5), 0, 0],
        [0,           0,          1, d5],
        [0,           0,          0, 1]
    ])

    # Chain
    T = np.eye(4)
    origins = [T[:3, 3].copy()]

    for A in [A1, A2, A3, A4, A5]:
        T = T @ A
        origins.append(T[:3, 3].copy())

    return np.array(origins)  # (6, 3) points
# -----------------------------------------------------------------------------------------------

# ---------------------- update function (right after forward kinimatics) -----------------------
def update_robot_visualization(app, subscriber):
    """Real-time 3D robot update"""
    try:
        joints_deg = np.rad2deg(subscriber.get_joints())
        points = forward_kinematics(joints_deg)
        ax = app.ax

        # Clear previous drawings
        if hasattr(app, 'robot_lines') and app.robot_lines is not None:
            app.robot_lines.remove()
        if hasattr(app, 'joint_scatter') and app.joint_scatter is not None:
            app.joint_scatter.remove()

        # Draw robot
        app.robot_lines = ax.plot(points[:,0], points[:,1], points[:,2],
                                  color='#00ddff', linewidth=6, solid_capstyle='round')[0]
        app.joint_scatter = ax.scatter(points[:,0], points[:,1], points[:,2],
                                       color='#ffaa00', s=120)
        # End-effector highlight
        ax.scatter(points[-1,0], points[-1,1], points[-1,2], color='red', s=200)

        app.canvas.draw_idle()
    except Exception as e:
        print(f"[Vis Error] {e}")

    app.after(50, lambda: update_robot_visualization(app, subscriber))
# ---------------------------------------------------------------------------------------------------

# ------------------------ 3D widget (in main function - right frame) -------------------------------
    fig = Figure(figsize=(9, 6), dpi=110, facecolor='#2b2b2b')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#1e1e1e')
    
    canvas = FigureCanvasTkAgg(fig, master=app)
    canvas.get_tk_widget().grid(row=2, column=0, columnspan=2, sticky='nsew', padx=12, pady=8)
    
    # Styling
    ax.set_xlabel('X (mm)', color='white')
    ax.set_ylabel('Y (mm)', color='white')
    ax.set_zlabel('Z (mm)', color='white')
    ax.set_title('5-DOF Robot Arm - Real Time', color='white', pad=15)
    ax.grid(True, alpha=0.3)
    
    # Wide view so base is in center and full 360° rotation is visible
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(0, 600)
    ax.view_init(elev=25, azim=-60)
    
    # Store references
    app.fig = fig
    app.ax = ax
    app.canvas = canvas
    app.robot_lines = None
    app.joint_scatter = None
# ---------------------------------------------------------------------------------------------------

# ----------------- calling the function (right after widget block ----------------------------------
    # Start real-time animation
    app.after(200, lambda: update_robot_visualization(app, subscriber))
# ---------------------------------------------------------------------------------------------------

