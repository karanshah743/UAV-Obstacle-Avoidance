import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from realsense_object.set_mode_node_pymavlink import set_flight_mode

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')

        # --- Declare parameters with defaults (overridden by YAML) ---
        self.declare_parameter('sub_distance_topic', 'object_distance')
        self.declare_parameter('depth_thresh_m', 1.0)
        self.declare_parameter('target_mode', 'LOITER')

        # --- Read parameters ---
        topic = self.get_parameter('sub_distance_topic').get_parameter_value().string_value
        self.threshold = self.get_parameter('depth_thresh_m').get_parameter_value().double_value
        self.target_mode = self.get_parameter('target_mode').get_parameter_value().string_value

        # --- Subscription ---
        self.subscription = self.create_subscription(Float32, topic, self.listener_callback, 10)

        self.mode_set = False
        self.get_logger().info(
            f"Listening on '{topic}', threshold={self.threshold} m, target_mode={self.target_mode}"
        )

    def listener_callback(self, msg: Float32):
        d = float(msg.data)

        if not math.isfinite(d) or d <= 0:
            return

        self.get_logger().info(f"closest-of-5: {d:.2f} m")

        if d < self.threshold and not self.mode_set:
            self.get_logger().warn(f"Obstacle too close ({d:.2f} m). Switching to {self.target_mode}…")
            if set_flight_mode(self.target_mode):
                self.mode_set = True
        elif d >= self.threshold and self.mode_set:
            self.get_logger().info(f"Safe distance restored ({d:.2f} m). Resetting gate.")
            self.mode_set = False


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


















# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from std_msgs.msg import Float32
# from pymavlink import mavutil

# # use your existing helper to change modes
# from realsense_object.set_mode_node_pymavlink import set_flight_mode

# MAV_MODE_FLAG_SAFETY_ARMED = 0x80  # 128

# class DistanceWithArmAltitudeGate(Node):
#     def __init__(self):
#         super().__init__('distance_with_arm_altitude_gate')

#         # ---------- Parameters ----------
#         self.declare_parameter('distance_topic', 'object_distance')
#         self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14550')  # shared UDP OK
#         self.declare_parameter('min_climb_after_arm_m', 5.0)          # gate opens after this climb
#         self.declare_parameter('hold_above_sec', 2.0)                  # must sustain for this long
#         self.declare_parameter('distance_threshold_m', 1.0)
#         self.declare_parameter('target_mode', 'LOITER')                # POSCTL/LOITER/etc.
#         self.declare_parameter('log_every_sec', 3.0)

#         self.distance_topic     = self.get_parameter('distance_topic').get_parameter_value().string_value
#         self.mavlink_url        = self.get_parameter('mavlink_url').get_parameter_value().string_value
#         self.min_climb_m        = float(self.get_parameter('min_climb_after_arm_m').value)
#         self.hold_above_sec     = float(self.get_parameter('hold_above_sec').value)
#         self.distance_threshold = float(self.get_parameter('distance_threshold_m').value)
#         self.target_mode        = self.get_parameter('target_mode').get_parameter_value().string_value
#         self.log_every_sec      = float(self.get_parameter('log_every_sec').value)

#         # ---------- State ----------
#         self.current_alt_m: float | None = None      # latest relative altitude (m)
#         self.armed: bool = False
#         self.armed_alt_m: float | None = None        # altitude captured at arming moment
#         self.altitude_ready: bool = False
#         self.mode_set: bool = False
#         self.last_above_time = None
#         self._last_wait_log = self.get_clock().now()

#         # ---------- ROS ----------
#         self.create_subscription(Float32, self.distance_topic, self._on_distance, 10)

#         # ---------- MAVLink ----------
#         self.master = None
#         self._connect_mavlink()
#         # Poll MAVLink + evaluate gate @ 10 Hz
#         self.timer = self.create_timer(0.1, self._poll_and_gate)

#         self.get_logger().info(
#             f"Gate requires: ARMED + climb ≥ {self.min_climb_m} m (hold {self.hold_above_sec}s). "
#             f"Then if distance < {self.distance_threshold} m → {self.target_mode}."
#         )

#     # --- MAVLink connection ---
#     def _connect_mavlink(self):
#         try:
#             self.get_logger().info(f"Connecting MAVLink: {self.mavlink_url} …")
#             self.master = mavutil.mavlink_connection(self.mavlink_url, autoreconnect=True, source_system=246)
#         except Exception as e:
#             self.get_logger().error(f"MAVLink connect failed: {e}")
#             self.master = None

#     # --- Poll MAVLink + update armed/alt + evaluate gate ---
#     def _poll_and_gate(self):
#         if self.master is None or self.master.port is None:
#             self._connect_mavlink()
#             return

#         try:
#             for _ in range(15):
#                 msg = self.master.recv_match(
#                     type=['HEARTBEAT', 'GLOBAL_POSITION_INT', 'VFR_HUD'],
#                     blocking=False
#                 )
#                 if msg is None:
#                     break

#                 t = msg.get_type()

#                 if t == 'HEARTBEAT':
#                     base_mode = int(getattr(msg, 'base_mode', 0))
#                     now_armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

#                     # detect arming edge
#                     if now_armed and not self.armed:
#                         self.armed = True
#                         # capture altitude at the instant of arming (fallback to 0 if unknown)
#                         self.armed_alt_m = self.current_alt_m if self.current_alt_m is not None else 0.0
#                         self.altitude_ready = False
#                         self.last_above_time = None
#                         self.mode_set = False
#                         self.get_logger().info(
#                             f"ARMED. Alt at arming = {self.armed_alt_m:.1f} m. Waiting for climb ≥ {self.min_climb_m} m."
#                         )

#                     # detect disarming edge
#                     elif not now_armed and self.armed:
#                         self.armed = False
#                         self.altitude_ready = False
#                         self.armed_alt_m = None
#                         self.last_above_time = None
#                         self.mode_set = False
#                         self.get_logger().warn("DISARMED. Gate closed; distance logic paused.")

#                 elif t == 'GLOBAL_POSITION_INT':
#                     # relative_alt is in millimeters above EKF origin/home
#                     self.current_alt_m = max(0.0, float(msg.relative_alt) / 1000.0)

#                 elif t == 'VFR_HUD':
#                     # alt in meters (AMSL). Only use if we have no relative altitude yet.
#                     if self.current_alt_m is None:
#                         self.current_alt_m = max(0.0, float(msg.alt))

#         except Exception as e:
#             self.get_logger().warn(f"MAVLink poll error: {e}")
#             return

#         # Evaluate the gate: must be armed AND climbed enough after the arming moment
#         now = self.get_clock().now()
#         if not self.armed:
#             # throttle logs
#             if (now - self._last_wait_log) > Duration(seconds=self.log_every_sec):
#                 self.get_logger().info("Gate closed: waiting for ARMED state.")
#                 self._last_wait_log = now
#             self.altitude_ready = False
#             return

#         if self.current_alt_m is None or self.armed_alt_m is None:
#             if (now - self._last_wait_log) > Duration(seconds=self.log_every_sec):
#                 self.get_logger().info("Gate closed: waiting for altitude data…")
#                 self._last_wait_log = now
#             self.altitude_ready = False
#             return

#         climb_since_arm = self.current_alt_m - self.armed_alt_m
#         if climb_since_arm >= self.min_climb_m:
#             if self.last_above_time is None:
#                 self.last_above_time = now
#             if (now - self.last_above_time) >= Duration(seconds=self.hold_above_sec):
#                 if not self.altitude_ready:
#                     self.altitude_ready = True
#                     self.get_logger().info(
#                         f"Gate OPEN (armed + climbed {climb_since_arm:.1f} m ≥ {self.min_climb_m} m)."
#                     )
#         else:
#             if self.altitude_ready:
#                 self.get_logger().warn(
#                     f"Gate CLOSED (climb fell to {climb_since_arm:.1f} m)."
#                 )
#             self.altitude_ready = False
#             self.last_above_time = None

#     # --- Distance logic (runs only when gate is open) ---
#     def _on_distance(self, msg: Float32):
#         d = float(msg.data)

#         if not (self.armed and self.altitude_ready):
#             now = self.get_clock().now()
#             if (now - self._last_wait_log) > Duration(seconds=self.log_every_sec):
#                 arm_txt = "ARMED" if self.armed else "DISARMED"
#                 alt_txt = "unknown" if self.current_alt_m is None else f"{self.current_alt_m:.1f} m"
#                 arm_alt_txt = "unknown" if self.armed_alt_m is None else f"{self.armed_alt_m:.1f} m"
#                 self.get_logger().info(
#                     f"Ignoring distance {d:.2f} m. Gate needs ARMED + climb ≥ {self.min_climb_m} m "
#                     f"(state={arm_txt}, alt={alt_txt}, alt@arm={arm_alt_txt})."
#                 )
#                 self._last_wait_log = now
#             return

#         if d < self.distance_threshold and not self.mode_set:
#             self.get_logger().warn(
#                 f"Obstacle {d:.2f} m < {self.distance_threshold:.2f} m → switching to {self.target_mode}…"
#             )
#             if set_flight_mode(self.target_mode):
#                 self.mode_set = True
#         elif d >= self.distance_threshold and self.mode_set:
#             self.get_logger().info(f"Safe distance restored ({d:.2f} m).")
#             self.mode_set = False


# def main(args=None):
#     rclpy.init(args=args)
#     node = DistanceWithArmAltitudeGate()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()














# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from std_msgs.msg import Float32
# from pymavlink import mavutil

# from realsense_object.set_mode_node_pymavlink import set_flight_mode

# MAV_MODE_FLAG_SAFETY_ARMED = 0x80
# AIRCRAFT_TYPES = {2, 3, 4, 5, 6, 27, 28, 29, 30}  # FW, Quad, Heli, VTOL, Rover/Boat (optional)

# class DistanceWithArmAltitudeGate(Node):
#     def __init__(self):
#         super().__init__('depth_subscriber')
 
#         # params (same as before)
#         self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14550')
#         self.declare_parameter('distance_topic', 'object_distance')
#         self.declare_parameter('min_climb_after_arm_m', 5.0)
#         self.declare_parameter('hold_above_sec', 2.0)
#         self.declare_parameter('distance_threshold_m', 1.0)
#         self.declare_parameter('target_mode', 'LOITER')
#         self.declare_parameter('autopilot_sysid', 0)   # 0=auto-detect

#         # state
#         self.current_alt_m = None
#         self.armed = False
#         self.armed_alt_m = None
#         self.altitude_ready = False
#         self.last_above_time = None
#         self.mode_set = False
#         self._last_wait_log = self.get_clock().now()
#         self.ap_sys = None
#         self.ap_comp = None

#         # ros
#         topic = self.get_parameter('distance_topic').get_parameter_value().string_value
#         self.create_subscription(Float32, topic, self._on_distance, 10)

#         # mavlink
#         self.master = mavutil.mavlink_connection(self.get_parameter('mavlink_url').get_parameter_value().string_value,
#                                                  autoreconnect=True, source_system=246)
#         self._detect_autopilot_endpoint()   # <-- important
#         self.timer = self.create_timer(0.1, self._poll_and_gate)

#     def _detect_autopilot_endpoint(self, timeout_s: float = 5.0):
#         """Pick the (sysid,compid) that is the actual AUTOPILOT."""
#         pin_sys = int(self.get_parameter('autopilot_sysid').value)
#         start = self.get_clock().now()
#         while (self.get_clock().now() - start).nanoseconds < timeout_s * 1e9:
#             hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
#             if hb is None:
#                 continue
#             s, c = hb.get_srcSystem(), hb.get_srcComponent()
#             if pin_sys and s != pin_sys:
#                 continue
#             # prefer component 1 (MAV_COMP_ID_AUTOPILOT1) and aircraft types
#             if c == 1 or int(getattr(hb, 'type', 0)) in AIRCRAFT_TYPES:
#                 self.ap_sys, self.ap_comp = s, c
#                 self.get_logger().info(f"Selected autopilot sysid={s} compid={c} type={getattr(hb,'type',-1)}")
#                 return
#         # fallback: lock to first heartbeat seen
#         if self.ap_sys is None:
#             hb = self.master.wait_heartbeat(timeout=2)
#             self.ap_sys, self.ap_comp = (hb.get_srcSystem(), hb.get_srcComponent()) if hb else (1, 1)
#             self.get_logger().warn(f"Autopilot not positively identified. Using sysid={self.ap_sys} compid={self.ap_comp}")

#     def _from_autopilot(self, msg) -> bool:
#         return (msg.get_srcSystem() == self.ap_sys) and (msg.get_srcComponent() == self.ap_comp)

#     def _poll_and_gate(self):
#         for _ in range(20):
#             m = self.master.recv_match(type=['HEARTBEAT', 'GLOBAL_POSITION_INT'], blocking=False)
#             if m is None:
#                 break
#             if not self._from_autopilot(m):
#                 continue

#             t = m.get_type()
#             if t == 'HEARTBEAT':
#                 now_armed = bool(int(getattr(m, 'base_mode', 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
#                 if now_armed and not self.armed:
#                     self.armed = True
#                     # only capture when we have RELATIVE altitude; otherwise wait
#                     self.armed_alt_m = self.current_alt_m if self.current_alt_m is not None else None
#                     self.altitude_ready = False
#                     self.last_above_time = None
#                     self.mode_set = False
#                     self.get_logger().info(f"ARMED (from autopilot hb). alt@arm={'n/a' if self.armed_alt_m is None else f'{self.armed_alt_m:.1f} m'}")
#                 elif not now_armed and self.armed:
#                     self.armed = False
#                     self.altitude_ready = False
#                     self.armed_alt_m = None
#                     self.last_above_time = None
#                     self.mode_set = False
#                     self.get_logger().warn("DISARMED (from autopilot hb). Gate closed.")

#             elif t == 'GLOBAL_POSITION_INT':
#                 # strictly relative altitude (mm) from AUTOPILOT only
#                 self.current_alt_m = max(0.0, float(m.relative_alt) / 1000.0)
#                 # if we armed earlier but had no rel-alt then, set it now at first rel-alt
#                 if self.armed and self.armed_alt_m is None:
#                     self.armed_alt_m = self.current_alt_m
#                     self.get_logger().info(f"alt@arm fixed using first rel-alt: {self.armed_alt_m:.1f} m")

#         # gate evaluation
#         if not self.armed or self.current_alt_m is None or self.armed_alt_m is None:
#             self.altitude_ready = False
#             return

#         climb = self.current_alt_m - self.armed_alt_m
#         hold = float(self.get_parameter('hold_above_sec').value)
#         min_climb = float(self.get_parameter('min_climb_after_arm_m').value)

#         now = self.get_clock().now()
#         if climb >= min_climb:
#             if self.last_above_time is None:
#                 self.last_above_time = now
#             if (now - self.last_above_time).nanoseconds >= hold * 1e9:
#                 self.altitude_ready = True
#         else:
#             self.altitude_ready = False
#             self.last_above_time = None

#     def _on_distance(self, msg: Float32):
#         if not (self.armed and self.altitude_ready):
#             return
#         d = float(msg.data)
#         thr = float(self.get_parameter('distance_threshold_m').value)
#         if d < thr and not self.mode_set:
#             if set_flight_mode(self.get_parameter('target_mode').get_parameter_value().string_value):
#                 self.mode_set = True
#         elif d >= thr and self.mode_set:
#             self.mode_set = False


# def main(args=None):
#     import rclpy
#     rclpy.init(args=args)
#     node = DistanceWithArmAltitudeGate()   # your class name
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()














# #!/usr/bin/env python3
# import math
# import time
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# from pymavlink import mavutil
# from realsense_object.set_mode_node_pymavlink import set_flight_mode

# MAV_MODE_FLAG_SAFETY_ARMED = 0x80
# AIRCRAFT_TYPES = {2, 3, 4, 5, 6, 27, 28, 29, 30}  # FW, Quad, Heli, VTOL, Rover/Boat (optional)

# class DistanceWithArmAltitudeGate(Node):
#     def __init__(self):
#         super().__init__('depth_subscriber')

#         # ----- Parameters -----
#         self.declare_parameter('mavlink_url', 'udp:127.0.0.1:14550')
#         self.declare_parameter('distance_topic', 'object_distance')
#         self.declare_parameter('min_climb_after_arm_m', 5.0)
#         self.declare_parameter('hold_above_sec', 2.0)
#         self.declare_parameter('distance_threshold_m', 10.0)
#         self.declare_parameter('target_mode', 'LOITER')
#         self.declare_parameter('autopilot_sysid', 0)  # 0 = auto-detect

#         # ----- State -----
#         self.current_alt_m = None
#         self.armed = False
#         self.armed_alt_m = None
#         self.altitude_ready = False
#         self.last_above_time = None
#         self.mode_set = False
#         self.ap_sys = None
#         self.ap_comp = None
#         self._last_log = 0.0

#         # ----- ROS -----
#         topic = self.get_parameter('distance_topic').value
#         self.create_subscription(Float32, topic, self._on_distance, 10)

#         # ----- MAVLink -----
#         url = self.get_parameter('mavlink_url').value
#         self.master = mavutil.mavlink_connection(url, autoreconnect=True, source_system=246)
#         self._detect_autopilot_endpoint()
#         self.timer = self.create_timer(0.1, self._poll_and_gate)  # 10 Hz

#         self.get_logger().info(
#             f"Subscribing to '{topic}', threshold={self.get_parameter('distance_threshold_m').value} m, "
#             f"gate: climb>={self.get_parameter('min_climb_after_arm_m').value} m for "
#             f"{self.get_parameter('hold_above_sec').value} s; target_mode={self.get_parameter('target_mode').value}"
#         )

#     # ---------- MAVLink helpers ----------
#     def _detect_autopilot_endpoint(self, timeout_s: float = 5.0):
#         """Pick (sysid,compid) for the AUTOPILOT by watching HEARTBEAT."""
#         pin_sys = int(self.get_parameter('autopilot_sysid').value)
#         end = time.time() + timeout_s
#         while time.time() < end:
#             hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
#             if hb is None:
#                 continue
#             s, c = hb.get_srcSystem(), hb.get_srcComponent()
#             if pin_sys and s != pin_sys:
#                 continue
#             if c == 1 or int(getattr(hb, 'type', 0)) in AIRCRAFT_TYPES:
#                 self.ap_sys, self.ap_comp = s, c
#                 self.get_logger().info(f"Selected autopilot sysid={s} compid={c} type={getattr(hb,'type',-1)}")
#                 return
#         if self.ap_sys is None:
#             hb = self.master.wait_heartbeat(timeout=2)
#             self.ap_sys, self.ap_comp = (hb.get_srcSystem(), hb.get_srcComponent()) if hb else (1, 1)
#             self.get_logger().warn(f"Autopilot not positively identified. Using sysid={self.ap_sys} compid={self.ap_comp}")

#     def _from_autopilot(self, msg) -> bool:
#         return (msg.get_srcSystem() == self.ap_sys) and (msg.get_srcComponent() == self.ap_comp)

#     def _poll_and_gate(self):
#         # Read a few MAVLink msgs per tick
#         for _ in range(20):
#             m = self.master.recv_match(type=['HEARTBEAT', 'GLOBAL_POSITION_INT'], blocking=False)
#             if m is None:
#                 break
#             if not self._from_autopilot(m):
#                 continue

#             if m.get_type() == 'HEARTBEAT':
#                 now_armed = bool(int(getattr(m, 'base_mode', 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
#                 if now_armed and not self.armed:
#                     self.armed = True
#                     self.armed_alt_m = self.current_alt_m if self.current_alt_m is not None else None
#                     self.altitude_ready = False
#                     self.last_above_time = None
#                     self.mode_set = False
#                     self.get_logger().info(
#                         f"ARMED. alt@arm={'n/a' if self.armed_alt_m is None else f'{self.armed_alt_m:.1f} m'}"
#                     )
#                 elif not now_armed and self.armed:
#                     self.armed = False
#                     self.altitude_ready = False
#                     self.armed_alt_m = None
#                     self.last_above_time = None
#                     self.mode_set = False
#                     self.get_logger().warn("DISARMED. Gate closed.")

#             elif m.get_type() == 'GLOBAL_POSITION_INT':
#                 # relative altitude in mm
#                 self.current_alt_m = max(0.0, float(m.relative_alt) / 1000.0)
#                 if self.armed and self.armed_alt_m is None:
#                     self.armed_alt_m = self.current_alt_m
#                     self.get_logger().info(f"alt@arm fixed from first rel-alt: {self.armed_alt_m:.1f} m")

#         # Evaluate altitude gate
#         if not self.armed or self.current_alt_m is None or self.armed_alt_m is None:
#             self.altitude_ready = False
#             return

#         min_climb = float(self.get_parameter('min_climb_after_arm_m').value)
#         hold_sec  = float(self.get_parameter('hold_above_sec').value)
#         climb = self.current_alt_m - self.armed_alt_m

#         if climb >= min_climb:
#             if self.last_above_time is None:
#                 self.last_above_time = time.time()
#             if (time.time() - self.last_above_time) >= hold_sec:
#                 if not self.altitude_ready:
#                     self.get_logger().info("Altitude gate OPEN.")
#                 self.altitude_ready = True
#         else:
#             if self.altitude_ready:
#                 self.get_logger().info("Altitude gate CLOSED (below min climb).")
#             self.altitude_ready = False
#             self.last_above_time = None

#     # ---------- Distance callback ----------
#     def _on_distance(self, msg: Float32):
#         # Only act when armed and altitude gate open
#         if not (self.armed and self.altitude_ready):
#             return

#         d = float(msg.data)
#         if not math.isfinite(d) or d <= 0.0:
#             return  # ignore invalid reads

#         thr = float(self.get_parameter('distance_threshold_m').value)
#         now = time.time()
#         if now - self._last_log > 0.5:
#             self.get_logger().info(f"closest-of-5: {d:.2f} m (thr={thr:.2f} m, mode_set={self.mode_set})")
#             self._last_log = now

#         if d < thr and not self.mode_set:
#             target = self.get_parameter('target_mode').value
#             self.get_logger().warn(f"Obstacle < {thr:.2f} m → switching to {target}…")
#             if set_flight_mode(target):     # should return quickly & non-blocking
#                 self.mode_set = True
#         elif d >= thr and self.mode_set:
#             self.get_logger().info("Safe distance restored.")
#             self.mode_set = False


# def main(args=None):
#     rclpy.init(args=args)
#     node = DistanceWithArmAltitudeGate()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
