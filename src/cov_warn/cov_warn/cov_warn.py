import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
import tkinter as tk

class CovMonitorNode(Node):
    def __init__(self, update_callback):
        super().__init__('cov_warn')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.update_callback = update_callback
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        cov = msg.position_covariance
        if hasattr(cov, "__len__") and len(cov) > 0:
            cov_value = cov[0]
        else:
            cov_value = None
        self.update_callback(cov_value)

class CovGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("(공분산 측정기)")
        self.geometry("300x300+0+0")  # 충분히 큰 정사각형, 좌상단
        self.resizable(False, False)
        self.label = tk.Label(self, text="Cov: N/A", font=("Arial", 12), justify="center")
        self.label.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)
        self.set_bg("gray")
        self.is_topmost = False

    def set_bg(self, color):
        self.configure(bg=color)
        self.label.configure(bg=color)

    def update_cov(self, cov_value):
        if cov_value is None:
            self.label.config(text="Cov: N/A", fg="gray", font=("Arial", 12))
            self.set_bg("gray")
            self._unset_topmost()
        else:
            if cov_value > 0.0004:
                color = "red"
                self._set_topmost()
                # "공분산이 높습니다! 비상! 비상!" 도배 (6줄)
                warning = "\n".join(["공분산이 높습니다! 비상! 비상!"] * 6)
                self.label.config(text=warning, fg="white", font=("Arial", 12, "bold"))
            else:
                color = "blue"
                self._unset_topmost()
                self.label.config(text=f"Cov: {cov_value:.6f}", fg=color, font=("Arial", 12))
            self.set_bg(color)

    def _set_topmost(self):
        if not self.is_topmost:
            self.lift()
            self.attributes("-topmost", True)
            self.is_topmost = True

    def _unset_topmost(self):
        if self.is_topmost:
            self.attributes("-topmost", False)
            self.is_topmost = False

def ros2_spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    rclpy.init()
    app = CovGUI()

    def update_cov_gui(cov_value):
        app.after(0, app.update_cov, cov_value)

    node = CovMonitorNode(update_cov_gui)
    t = threading.Thread(target=ros2_spin_thread, args=(node,), daemon=True)
    t.start()
    app.mainloop()

if __name__ == "__main__":
    main()
