#!/usr/bin/env python3

import json
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros


DEFAULT_OUTPUT_PATH = Path("config/recorded_robot_poses.json")
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_TOOL_FRAME = "wrist3_link"


class PoseRecorderNode(Node):
    def __init__(self):
        super().__init__("pose_recorder_gui_node")

        self.latest_joint_state = None
        self.joint_state_lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def get_joint_state_snapshot(self):
        with self.joint_state_lock:
            if self.latest_joint_state is None:
                return None

            return {
                "names": list(self.latest_joint_state.name),
                "positions": [float(x) for x in self.latest_joint_state.position],
                "velocities": [float(x) for x in self.latest_joint_state.velocity],
                "efforts": [float(x) for x in self.latest_joint_state.effort],
            }

    def get_transform_snapshot(self, base_frame, tool_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            return None, str(exc)

        t = transform.transform.translation
        q = transform.transform.rotation

        return {
            "parent_frame": base_frame,
            "child_frame": tool_frame,
            "translation_m": {
                "x": float(t.x),
                "y": float(t.y),
                "z": float(t.z),
            },
            "rotation_xyzw": {
                "x": float(q.x),
                "y": float(q.y),
                "z": float(q.z),
                "w": float(q.w),
            },
        }, None


class PoseRecorderGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node

        self.output_path = DEFAULT_OUTPUT_PATH
        self.poses = {}

        self.root.title("FAIRINO Pose Recorder")
        self.root.geometry("980x720")

        self.pose_name_var = tk.StringVar()
        self.base_frame_var = tk.StringVar(value=DEFAULT_BASE_FRAME)
        self.tool_frame_var = tk.StringVar(value=DEFAULT_TOOL_FRAME)
        self.status_var = tk.StringVar(value="Waiting for /joint_states and TF...")
        self.output_path_var = tk.StringVar(value=str(self.output_path))

        self._build_ui()
        self._load_existing_poses()
        self._refresh_live_display_loop()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Pose name:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.pose_name_var, width=36).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(top, text="Base frame:").grid(row=1, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.base_frame_var, width=24).grid(row=1, column=1, padx=6, sticky="w")

        ttk.Label(top, text="Tool frame:").grid(row=2, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.tool_frame_var, width=24).grid(row=2, column=1, padx=6, sticky="w")

        ttk.Label(top, text="Output file:").grid(row=3, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.output_path_var, width=72).grid(row=3, column=1, padx=6, sticky="w")

        ttk.Label(top, text="Notes:").grid(row=4, column=0, sticky="nw")
        self.notes_text = tk.Text(top, height=3, width=72)
        self.notes_text.grid(row=4, column=1, padx=6, pady=4, sticky="w")

        button_row = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        button_row.pack(fill=tk.X)

        ttk.Button(button_row, text="Save Current Pose", command=self.save_current_pose).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="Reload File", command=self._load_existing_poses).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="Delete Selected Pose", command=self.delete_selected_pose).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="Copy Selected JSON", command=self.copy_selected_pose_json).pack(side=tk.LEFT, padx=4)

        live_frame = ttk.LabelFrame(self.root, text="Live Robot State", padding=10)
        live_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.live_text = tk.Text(live_frame, height=16, width=120)
        self.live_text.pack(fill=tk.BOTH, expand=True)

        saved_frame = ttk.LabelFrame(self.root, text="Saved Poses", padding=10)
        saved_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.pose_list = tk.Listbox(saved_frame, height=10)
        self.pose_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(saved_frame, orient=tk.VERTICAL, command=self.pose_list.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.pose_list.config(yscrollcommand=scrollbar.set)

        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor="w", padding=5)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

    def _load_existing_poses(self):
        self.output_path = Path(self.output_path_var.get()).expanduser()

        if not self.output_path.exists():
            self.poses = {}
            self._refresh_pose_list()
            self.status_var.set(f"No existing pose file yet: {self.output_path}")
            return

        try:
            data = json.loads(self.output_path.read_text())
            self.poses = data.get("poses", {})
            self._refresh_pose_list()
            self.status_var.set(f"Loaded {len(self.poses)} poses from {self.output_path}")
        except Exception as exc:
            messagebox.showerror("Load error", f"Failed to load pose file:\n{exc}")

    def _write_pose_file(self):
        self.output_path = Path(self.output_path_var.get()).expanduser()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "schema": "fairino_pose_recorder_v1",
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "poses": self.poses,
        }

        self.output_path.write_text(json.dumps(data, indent=2))

    def _refresh_pose_list(self):
        self.pose_list.delete(0, tk.END)
        for name in sorted(self.poses.keys()):
            self.pose_list.insert(tk.END, name)

    def _get_selected_pose_name(self):
        selected = self.pose_list.curselection()
        if not selected:
            return None
        return self.pose_list.get(selected[0])

    def save_current_pose(self):
        name = self.pose_name_var.get().strip()
        if not name:
            messagebox.showwarning("Missing name", "Enter a pose name first.")
            return

        joint_state = self.ros_node.get_joint_state_snapshot()
        if joint_state is None:
            messagebox.showerror("No joint state", "No /joint_states received yet.")
            return

        base_frame = self.base_frame_var.get().strip()
        tool_frame = self.tool_frame_var.get().strip()

        transform, error = self.ros_node.get_transform_snapshot(base_frame, tool_frame)
        if transform is None:
            messagebox.showerror("No TF", f"Could not read TF {base_frame} -> {tool_frame}:\n{error}")
            return

        notes = self.notes_text.get("1.0", tk.END).strip()

        self.poses[name] = {
            "recorded_at": datetime.now().isoformat(timespec="seconds"),
            "notes": notes,
            "joint_state": joint_state,
            "tool_pose": transform,
        }

        self._write_pose_file()
        self._refresh_pose_list()
        self.status_var.set(f"Saved pose '{name}' to {self.output_path}")

    def delete_selected_pose(self):
        name = self._get_selected_pose_name()
        if not name:
            messagebox.showwarning("No selection", "Select a pose to delete.")
            return

        if not messagebox.askyesno("Confirm delete", f"Delete pose '{name}'?"):
            return

        self.poses.pop(name, None)
        self._write_pose_file()
        self._refresh_pose_list()
        self.status_var.set(f"Deleted pose '{name}'")

    def copy_selected_pose_json(self):
        name = self._get_selected_pose_name()
        if not name:
            messagebox.showwarning("No selection", "Select a pose to copy.")
            return

        payload = json.dumps({name: self.poses[name]}, indent=2)
        self.root.clipboard_clear()
        self.root.clipboard_append(payload)
        self.status_var.set(f"Copied pose '{name}' JSON to clipboard")

    def _refresh_live_display_loop(self):
        joint_state = self.ros_node.get_joint_state_snapshot()
        base_frame = self.base_frame_var.get().strip()
        tool_frame = self.tool_frame_var.get().strip()
        transform, tf_error = self.ros_node.get_transform_snapshot(base_frame, tool_frame)

        lines = []

        if joint_state is None:
            lines.append("No /joint_states received yet.")
        else:
            lines.append("Joint positions:")
            for name, position in zip(joint_state["names"], joint_state["positions"]):
                lines.append(f"  {name}: {position:.6f} rad")

        lines.append("")
        lines.append(f"TF {base_frame} -> {tool_frame}:")

        if transform is None:
            lines.append(f"  unavailable: {tf_error}")
        else:
            p = transform["translation_m"]
            q = transform["rotation_xyzw"]
            lines.append(f"  x:  {p['x']:.6f} m")
            lines.append(f"  y:  {p['y']:.6f} m")
            lines.append(f"  z:  {p['z']:.6f} m")
            lines.append(f"  qx: {q['x']:.6f}")
            lines.append(f"  qy: {q['y']:.6f}")
            lines.append(f"  qz: {q['z']:.6f}")
            lines.append(f"  qw: {q['w']:.6f}")

        self.live_text.delete("1.0", tk.END)
        self.live_text.insert(tk.END, "\n".join(lines))

        self.root.after(500, self._refresh_live_display_loop)


def main():
    rclpy.init()
    ros_node = PoseRecorderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = PoseRecorderGUI(root, ros_node)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
