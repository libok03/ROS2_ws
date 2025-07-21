import tkinter as tk
from tkinter import simpledialog, messagebox
import os
import subprocess
import sys
import signal
import psutil

# 환경설정
WORKSPACE_PATH = "/home/libok/dev_ws"
SETUP_BASH = os.path.join(WORKSPACE_PATH, "install/setup.bash")

TERMINAL_CONFIGS = [
    ("velodyne", "velodyne-all-nodes-VLP32C-launch.py", "launch"),
    ("xsens_mti_ros2_driver", "xsens_mti_node", "launch"),
    ("erp42_communication", "erp42.launch.py", "launch"),
    ("ublox_gps", "ublox_gps_node", "launch"),
    ("ntrip", "ntrip_launch.py", "launch"),
    ("robot_localization", "ekf.launch.py", "launch"),
    ("robot_localization", "navsat_transform.launch.py", "launch"),
    ("usb_cam", "camera.launch.py", "launch"),
    ("localization", "rotate_yaw", "run"),
] + [("dummy", "dummy", "run")] * (15 - 9)

class TerminalPanel(tk.Frame):
    def __init__(self, master, pkg, target, cmd_type, idx):
        super().__init__(master, bd=2, relief=tk.GROOVE)
        self.pkg = pkg
        self.target = target
        self.cmd_type = cmd_type
        self.idx = idx
        self.xterm_process = None

        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1, minsize=200)

        cmd_text = f"{pkg}\n{target} ({cmd_type})"
        file_label = tk.Label(self, text=cmd_text, font=("Arial", 11, "bold"),
                              bg="black", fg="lime")
        file_label.grid(row=0, column=0, sticky="ew", padx=1, pady=(1, 0))

        self.terminal_frame = tk.Frame(self, bg="black")
        self.terminal_frame.grid(row=1, column=0, sticky="nsew", padx=1, pady=1)

        btn_frame = tk.Frame(self)
        btn_frame.grid(row=2, column=0, sticky="ew", padx=1, pady=1)

        self.run_btn = tk.Button(btn_frame, text="실행", bg="lightgreen", command=self.launch_ros2)
        self.run_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.stop_btn = tk.Button(btn_frame, text="중단", bg="tomato",
                                  command=self.stop_ros2, state="disabled")
        self.stop_btn.pack(side=tk.LEFT, expand=True, fill=tk.X)

    def launch_ros2(self):
        if self.xterm_process:
            return

        if self.cmd_type == "launch":
            cmd = f"ros2 launch {self.pkg} {self.target}"
        else:
            cmd = f"ros2 run {self.pkg} {self.target}"

        full_cmd = f". {SETUP_BASH} && exec {cmd}"

        def spawn_xterm():
            frame_id = self.terminal_frame.winfo_id()
            self.xterm_process = subprocess.Popen(
                [
                    "xterm",
                    "-hold",
                    "-into", str(frame_id),
                    "-geometry", "80x12",
                    "-fa", "Terminus",
                    "-fs", "8",
                    "-e", "bash", "-i", "-c", full_cmd
                ],
                preexec_fn=os.setsid
            )
            self.run_btn.config(state="disabled")
            self.stop_btn.config(state="normal")

        self.terminal_frame.after(100, spawn_xterm)

    def stop_ros2(self):
        if self.xterm_process:
            try:
                os.killpg(os.getpgid(self.xterm_process.pid), signal.SIGINT)
            except Exception as e:
                print(f"[ERROR] 강제 종료 실패: {e}")
            finally:
                self.xterm_process = None

        if self.cmd_type == "launch":
            pkill_cmd = f"pkill -f 'ros2 launch {self.pkg} {self.target}'"
            try:
                subprocess.run(pkill_cmd, shell=True)
            except Exception as e:
                print(f"[ERROR] pkill 실패: {e}")

            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'])
                    if f"ros2 launch {self.pkg} {self.target}" in cmdline:
                        os.kill(proc.info['pid'], signal.SIGKILL)
                except Exception:
                    pass
        else:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'])
                    if f"ros2 run {self.pkg} {self.target}" in cmdline:
                        os.kill(proc.info['pid'], signal.SIGKILL)
                except Exception:
                    pass

        self.run_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

class ROS2MultiPanelGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS2 15-패널 런치/런 GUI")
        self.root.geometry("1920x1000")

        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        panel_frame = tk.Frame(main_frame)
        panel_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(panel_frame, borderwidth=0, highlightthickness=0)
        self.scrollbar = tk.Scrollbar(
            panel_frame, orient="vertical", command=self.canvas.yview, width=10
        )
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
        self.scrollbar.grid(row=0, column=1, sticky="ns", padx=0, pady=0)

        panel_frame.grid_rowconfigure(0, weight=1)
        panel_frame.grid_columnconfigure(0, weight=1)

        self.scrollable_frame = tk.Frame(self.canvas)
        self.scrollable_frame_id = self.canvas.create_window(
            (0, 0), window=self.scrollable_frame, anchor="nw"
        )

        def _on_configure(event):
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))
            self.canvas.itemconfig(self.scrollable_frame_id, width=event.width)

        self.canvas.bind("<Configure>", _on_configure)

        self.panels = []
        for i in range(3):
            self.scrollable_frame.grid_rowconfigure(i, weight=1, minsize=0)
            for j in range(5):
                self.scrollable_frame.grid_columnconfigure(j, weight=1, minsize=0)
                idx = i * 5 + j
                if idx < len(TERMINAL_CONFIGS):
                    pkg, target, cmd_type = TERMINAL_CONFIGS[idx]
                    panel = TerminalPanel(self.scrollable_frame, pkg, target, cmd_type, idx)
                    panel.grid(row=i, column=j, sticky="nsew", padx=1, pady=1)
                    self.panels.append(panel)

        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _on_mousewheel(self, event):
        self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _launch_tool(self, command, geometry="100x30"):
        subprocess.Popen(
            [
                "xterm",
                "-hold",
                "-geometry",
                geometry,
                "-fa",
                "Monospace",
                "-fs",
                "10",
                "-e",
                "bash",
                "-i",
                "-c",
                f". {SETUP_BASH} && {command}",
            ]
        )

    def _launch_topic_echo(self):
        topic = simpledialog.askstring("Topic Echo", "토픽 이름을 입력하세요:")
        if topic:
            self._launch_tool(f"ros2 topic echo {topic}", geometry="100x40")

    def _launch_node_list(self):
        self._launch_tool("watch -n 0.5 ros2 node list", geometry="100x20")
        
    def _get_topic_list(self):
        try:
            result = subprocess.run(
                f". {SETUP_BASH} && ros2 topic list",
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                topics = [topic for topic in result.stdout.splitlines() if topic.strip()]
                return topics
            return []
        except Exception as e:
            print(f"토픽 목록 가져오기 실패: {e}")
            return []

    def _launch_topic_echo_select(self):
        topics = self._get_topic_list()
        if not topics:
            messagebox.showinfo("Info", "활성화된 토픽이 없습니다.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("토픽 선택")
        dialog.geometry("400x300")
        dialog.transient(self.root)
        dialog.grab_set()
        
        frame = tk.Frame(dialog)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        scrollbar = tk.Scrollbar(frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        listbox = tk.Listbox(
            frame, 
            yscrollcommand=scrollbar.set,
            font=("Arial", 12),
            selectbackground="#4CAF50",
            selectmode=tk.SINGLE
        )
        listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=listbox.yview)
        
        for topic in topics:
            listbox.insert(tk.END, topic)
        
        def on_select():
            selection = listbox.curselection()
            if selection:
                topic = listbox.get(selection[0])
                dialog.destroy()
                self._launch_tool(f"ros2 topic echo {topic}", geometry="100x40")
        
        btn_frame = tk.Frame(dialog)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        select_btn = tk.Button(
            btn_frame, 
            text="선택", 
            bg="#4CAF50", 
            fg="white",
            font=("Arial", 10, "bold"),
            command=on_select
        )
        select_btn.pack(side=tk.RIGHT)

if __name__ == "__main__":
    # cov_warn 노드 자동 실행 (백그라운드)
    try:
        cov_warn_proc = subprocess.Popen(
            ["ros2", "run", "cov_warn", "cov_warn"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
    except Exception as e:
        print(f"[경고] cov_warn 자동 실행 실패: {e}", file=sys.stderr)

    root = tk.Tk()
    app = ROS2MultiPanelGUI(root)

    # 하단 도구 버튼 프레임
    button_frame = tk.Frame(root)
    button_frame.pack(fill=tk.X, padx=5, pady=5)

    def create_tool_button(parent, text, command, bg_color):
        return tk.Button(
            parent,
            text=text,
            bg=bg_color,
            fg="white",
            font=("Arial", 10, "bold"),
            command=command,
            height=2,
        )

    buttons = [
        ("Rviz2", "#4CAF50", lambda: app._launch_tool("rviz2")),
        ("rqt_graph", "#2196F3", lambda: app._launch_tool("rqt_graph")),
        ("rqt", "#9C27B0", lambda: app._launch_tool("rqt")),
        ("Topic Echo (Select)", "#FF9800", app._launch_topic_echo_select),
        ("Node List", "#607D8B", app._launch_node_list),
    ]

    for text, color, cmd in buttons:
        btn = create_tool_button(button_frame, text, cmd, color)
        btn.pack(side=tk.LEFT, padx=3, pady=3, fill=tk.X, expand=True)

    root.mainloop()
