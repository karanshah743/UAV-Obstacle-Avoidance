# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import subprocess
# import select
# import threading
           

# class RealSensePublisher(Node):
#     def __init__(self):
#         super().__init__('realsense_depth_publisher')
#         self.publisher_ = self.create_publisher(Float32, 'object_distance', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         # RealSense setup
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         self.pipeline.start(config)

#         # self.align = rs.align(rs.stream.color)
#         self.get_logger().info("RealSense streaming started with visualization.")
    
#     def timer_callback(self):
#         frames = self.pipeline.wait_for_frames()
        
#         if not frames:
#             self.get_logger().warning("No frames received.")
#             return
        
#         # aligned_frames = self.align.process(frames)

#         depth_frame = frames.get_depth_frame()
#         # color_frame = aligned_frames.get_color_frame()

#         if not depth_frame:
#             self.get_logger().warning("Missing frames.")
#             return

#         # Convert to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         # color_image = np.asanyarray(color_frame.get_data())
        
#         # Convert depth to 8-bit (0-255) for visualization
#         depth_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)

#         # Apply color map (e.g., COLORMAP_JET)
#         depth_colormap = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

#         # Get center point distance
#         h, w = depth_image.shape
#         center_x, center_y = w // 2, h // 2
#         depth = depth_frame.get_distance(center_x, center_y)

#         if depth > 0:
#             msg = Float32()
#             msg.data = depth
#             self.publisher_.publish(msg)
#             # self.get_logger().info(f'Depth at center: {depth:.2f} m')

#             # Draw center point and depth text
#             cv2.circle(depth_colormap, (center_x, center_y), 5, (0, 255, 0), -1)
#             cv2.putText(depth_colormap, f"{depth:.2f} m", (center_x + 10, center_y - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
#         # Show the image
#         cv2.imshow("RealSense RGB + Depth", depth_colormap)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             self.get_logger().info("Quitting visualization.")
#             rclpy.shutdown()

#     def destroy_node(self):
#         self.pipeline.stop()
#         cv2.destroyAllWindows()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = RealSensePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Interrupted by user.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()










import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

def make_gst_pipeline(width, height, fps, host, port, use_hw=True, bitrate=4000000):
    """
    OpenCV BGR -> videoconvert (CPU) -> [nvvidconv -> NVMM/NV12] -> enc -> rtph264pay -> udpsink
    """
    base = (
        f"appsrc is-live=true block=true format=time "
        f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 "
        f"! queue leaky=downstream max-size-buffers=2 "
        f"! videoconvert "
    )
    if use_hw:
        enc = (
            "! nvvidconv "
            "! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc insert-sps-pps=true iframeinterval=30 control-rate=1 bitrate={bitrate} "
            "! h264parse "
        )
    else:
        enc = (
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 "
            "! h264parse "
        )
    tail = (
        "! rtph264pay pt=96 config-interval=1 "
        f"! udpsink host={host} port={port} sync=false async=false"
    )
    return base + enc + tail

def make_local_rec_pipeline(width, height, fps, use_hw=True, bitrate=4000000, out_dir="/home/nvidia/flight", ts=""):
    """
    Return a GStreamer pipeline string for splitmux MP4 recording.
    Tries HW encoder first; if unavailable, returns software x264 pipeline.
    """
    # splitmux settings: 120s segments, ~2GB max; robust muxing
    seg = (f"splitmuxsink location={out_dir}/seg_{ts}_%05d.mp4 "
           f"max-size-time=120000000000 max-size-bytes=2147483648 "
           f"use-robust-muxing=true muxer=mp4mux")
    base = "appsrc is-live=true block=true format=time " \
           f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! " \
           "queue leaky=downstream max-size-buffers=2 ! videoconvert "

    if use_hw:
        # HW path (Jetson). If it fails to open we'll try SW below.
        return base + "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 " \
               f"! nvv4l2h264enc insert-sps-pps=true iframeinterval=30 control-rate=1 bitrate={bitrate} " \
               "! h264parse ! " + seg

    # Software fallback
    return base + "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 " \
           "! h264parse ! " + seg


def grid_boxes(h, w, rows=3, cols=3):
    boxes = {}
    for r in range(1, rows+1):
        r0 = (r-1)*h//rows; r1 = r*h//rows
        for c in range(1, cols+1):
            c0 = (c-1)*w//cols; c1 = c*w//cols
            boxes[(r,c)] = (r0, r1, c0, c1)
    return boxes

def eval_cell(depth_m, box, thresh_m, min_cov, min_valid_px=150, low_pct=15):
    r0, r1, c0, c1 = box
    roi = depth_m[r0:r1, c0:c1]
    valid = roi > 0
    n_valid = int(np.count_nonzero(valid))
    if n_valid < min_valid_px:
        # Not enough data -> ignore this cell
        return False, np.nan, 0.0, np.nan, n_valid, np.nan

    rv = roi[valid]

    # Basic sanity clipping (drop extreme outliers)
    lo, hi = np.percentile(rv, [1, 99])
    rv = rv[(rv >= lo) & (rv <= hi)]
    if rv.size < min_valid_px:
        return False, np.nan, 0.0, np.nan, rv.size, np.nan

    close_frac = float(np.mean(rv < thresh_m))
    med = float(np.median(rv))
    # robust "near" estimator instead of absolute minimum
    near = float(np.percentile(rv, low_pct))

    obstacle = (close_frac >= min_cov)
    return obstacle, med, close_frac, near, n_valid, lo

class RealSenseStreamer(Node):
    def __init__(self):
        super().__init__('realsense_streamer')
        
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('depth_thresh_m', 1.0)
        self.declare_parameter('min_coverage', 0.15)
        self.declare_parameter('max_range_m', 10.0)
        self.declare_parameter('show_local_preview', False)

        self.declare_parameter('gcs_ip', '192.168.0.65')
        self.declare_parameter('gcs_port', 5600)
        self.declare_parameter('use_nvv4l2', False)
        self.declare_parameter('bitrate', 4000000)
        
        W   = self.get_parameter('width').get_parameter_value().integer_value
        H   = self.get_parameter('height').get_parameter_value().integer_value
        FPS = self.get_parameter('fps').get_parameter_value().integer_value
        
        self.DEPTH_THRESH_M = float(self.get_parameter('depth_thresh_m').value)
        self.MIN_COVERAGE   = float(self.get_parameter('min_coverage').value)
        self.MAX_RANGE_M = float(self.get_parameter('max_range_m').value)
        self.SHOW_LOCAL_PREVIEW = bool(self.get_parameter('show_local_preview').value)

        GCS_IP   = str(self.get_parameter('gcs_ip').value)
        GCS_PORT = int(self.get_parameter('gcs_port').value)
        USE_NVV4L2 = bool(self.get_parameter('use_nvv4l2').value)
        BITRATE    = int(self.get_parameter('bitrate').value)

        # ROS pub (closest distance across the 5 target cells)
        self.pub_dist = self.create_publisher(Float32, 'object_distance', 10)
        self.last_closest = self.MAX_RANGE_M   # remember last good value
        self.timer = self.create_timer(1.0 / FPS, self.loop)

        # RealSense setup
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
        prof = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()

        # Light depth smoothing
        self.hf, self.tf, self.sf = rs.hole_filling_filter(), rs.temporal_filter(), rs.spatial_filter()

        # GStreamer writer
        gst = make_gst_pipeline(W, H, FPS, GCS_IP, GCS_PORT, USE_NVV4L2, BITRATE)
        self.out = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, FPS, (W, H), True)
        if not self.out.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline. "
                                    "If on Jetson, ensure OpenCV has GStreamer; otherwise set USE_NVV4L2=False.")
            raise RuntimeError("GStreamer pipeline open failed")
        
        out_dir = "/home/nvidia/uoa_flight"
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")

        gst_local = make_local_rec_pipeline(W, H, FPS, use_hw=True, bitrate=BITRATE, out_dir=out_dir, ts=ts)
        self.recorder = cv2.VideoWriter(gst_local, cv2.CAP_GSTREAMER, 0,
                                        FPS, (W, H), True)
        
        if not self.recorder.isOpened():
            self.get_logger().warn("HW recorder failed; falling back to software x264enc.")
            gst_local = make_local_rec_pipeline(W, H, FPS, use_hw=False, bitrate=BITRATE, out_dir=out_dir, ts=ts)
            self.recorder = cv2.VideoWriter(gst_local, cv2.CAP_GSTREAMER, 0, FPS, (W, H), True)
        
        if not self.recorder.isOpened():
            self.get_logger().error("Failed to open local recorder pipeline.")
            self.recorder = None
        else:
            self.get_logger().info(f"Recording MP4 segments to {out_dir} (rotating every ~120 s).")


        # Precompute cell boxes & target cells (row 2 & col 2)
        self.boxes = grid_boxes(H, W)
        self.targets = [(2,1), (2,2), (2,3), (1,2), (3,2)]

        self.last_fps_t, self.fps_cnt, self.fps = time.time(), 0, 0.0
        self.get_logger().info(f"Streaming to udp://{GCS_IP}:{GCS_PORT} (open QGC: UDP h.264).")

    def loop(self):
        try:
            frames = self.pipe.wait_for_frames()
            frames = self.align.process(frames)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                return

            # Smooth depth
            d = self.sf.process(d); d = self.tf.process(d); d = self.hf.process(d)

            depth_u16 = np.asanyarray(d.get_data())
            depth_m   = depth_u16 * self.scale
            color     = np.asanyarray(c.get_data())    
            
            overall = False
            nears, meds = [], []   # collect robust nears & medians from target cells

            for (r, cidx), (r0, r1, c0, c1) in self.boxes.items():
                is_target = (r, cidx) in self.targets
                if is_target:
                    obstacle, med, frac, near, n_valid, _ = eval_cell(
                        depth_m, (r0, r1, c0, c1),
                        self.DEPTH_THRESH_M, self.MIN_COVERAGE,
                        min_valid_px=150, low_pct=15
                    )
                    if np.isfinite(near):
                        nears.append(near)  # <-- collect regardless of obstacle
                    if np.isfinite(med):
                        meds.append(med)
                    if obstacle:
                        overall = True
                    col = (0, 0, 255) if obstacle else (0, 255, 0)
                    label = f"{r},{cidx} md:{med:.2f}m c%:{frac*100:.0f}"
                else:
                    roi = depth_m[r0:r1, c0:c1]
                    valid = roi[roi > 0]
                    med = float(np.median(valid)) if valid.size else np.nan
                    col = (180, 180, 180)
                    label = f"{r},{cidx} md:{med:.2f}m"

                cv2.rectangle(color, (c0, r0), (c1, r1), col, 2 if is_target else 1)
                cv2.putText(color, label, (c0 + 6, r0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (255, 255, 255), 1, cv2.LINE_AA)

            # Decide closest-of-5 with robust fallbacks (NO NaN publishing)
            if nears:
                closest = float(np.min(nears))
                self.last_closest = closest
            elif meds:
                # if no 15th-percentile available, fall back to median across targets
                closest = float(np.median(meds))
                self.last_closest = closest
            else:
                # no valid depth this frame → use last good value (bounded)
                closest = float(self.last_closest)

            # Always publish a finite number (meters)
            self.pub_dist.publish(Float32(data=closest))
            # self.get_logger().info(f"distance is {closest:.2f}")

            # # You can keep obstacle HUD logic separate
            # if closest < DEPTH_THRESH_M:
            #     self.get_logger().info(f"Obstacle-like proximity: {closest:.2f} m")



            # HUD + FPS
            self.fps_cnt += 1
            now = time.time()
            if now - self.last_fps_t >= 0.5:
                self.fps = self.fps_cnt / (now - self.last_fps_t)
                self.last_fps_t, self.fps_cnt = now, 0
            status = "OBSTACLE" if overall else "CLEAR"
            hud = f"{status} | TH:{self.DEPTH_THRESH_M}m COV:{int(self.MIN_COVERAGE*100)}% | FPS:{self.fps:.1f}"
            cv2.putText(color, hud, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255) if overall else (0, 255, 0), 2, cv2.LINE_AA)

            # Stream to QGC
            self.out.write(color)
            
            if self.recorder:
                self.recorder.write(color)


            if self.SHOW_LOCAL_PREVIEW:
                cv2.imshow("TX Preview", color)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")

    def destroy_node(self):
        try:
            self.out.release()
        except Exception:
            pass
        try:
            self.pipe.stop()
        except Exception:
            pass
        try:
            if self.recorder:
                self.recorder.release()
        except Exception:
            pass
        if self.SHOW_LOCAL_PREVIEW:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

