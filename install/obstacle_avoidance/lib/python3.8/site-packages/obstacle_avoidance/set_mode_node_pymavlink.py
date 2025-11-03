from pymavlink import mavutil #-- karan code
import time
import math

def set_flight_mode(mode):
    try:
        # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        master.wait_heartbeat(timeout=5)
        master.target_system = master.target_system or 1
        master.target_component = master.target_component or 1
        time.sleep(0.5)
        
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        
        base = msg.base_mode
        custom = msg.custom_mode
        sub = msg.system_status

        # current_mode = master.flightmode or 'LOITER'
        current_mode = mavutil.mode_string_v10(msg)
        print(f'[INFO] Current flight mode: {mode}')

        mode_map = master.mode_mapping()

        if mode not in mode_map:
            print(f'[WARNING] Mode "{mode}" not in mode map.')
            return

        else:
            print(f'[INFO] Switching to {mode} mode...')
            base_mode, main_mode, sub_mode = mode_map[mode]
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                base_mode, main_mode, sub_mode,
                0, 0, 0, 0
            )

            # 2. Send velocity=0 and yaw=+30°
            # yaw_deg = 30
            # # yaw_rad = math.radians(yaw_deg)
            # yaw_rate = math.radians(15)  # rotate right at 15 deg/s

            # print('[INFO] Holding position and rotating 30°...')
            # try:
            #     yaw_rad = math.radians(30)
            #     for _ in range(20):  # 2 seconds at 10 Hz
            #         timestamp_ms = int((time.time() * 1000) % 4294967295)
            #         master.mav.set_position_target_local_ned_send(
            #             timestamp_ms,
            #             master.target_system,
            #             master.target_component,
            #             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            #             0b0000011111000111,  # Bitmask: control vx, vy, vz, yaw
            #             0, 0, 0,              # Position (ignored)
            #             0, 0, 0,              # vx, vy, vz = hold
            #             0, 0, 0,              # accel (ignored)
            #             yaw_rad,              # yaw (absolute)
            #             0.0                   # yaw rate
            #         )
            #         time.sleep(0.3)
            #     print('Rotation completed.')

            # except Exception as e:
            #     print(f'[WARNING] Rotation command failed: {e}')

            # time.sleep(3)
            
            # print('[INFO] Reverting to previous mode:', current_mode)
            # r_base, r_main, r_sub = mode_map[current_mode]
            # master.mav.command_long_send(
            #     master.target_system,
            #     master.target_component,
            #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            #     0,
            #     r_base, r_main, r_sub,
            #     0, 0, 0, 0
            # ) 
            # return True

        # Handle regular mode change
        # if mode not in mode_map:
        #     print(f'[ERROR] Mode "{mode}" not supported.')
        #     return False

        # base_mode, main_mode, sub_mode = mode_map[mode]
        # master.mav.command_long_send(
        #     master.target_system,
        #     master.target_component,
        #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        #     0,
        #     base_mode, main_mode, sub_mode,
        #     0, 0, 0, 0
        # )

        # print('Working')
        # print(f'[INFO] Switched from {current_mode} → {mode}')
        return True

    except Exception as e:
        print(f'[ERROR] Failed to set mode: {e}')
        return False













# from pymavlink import mavutil
# import time, math, threading

# def set_flight_mode(mode) -> bool:
#     try:
#         master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
#         master.wait_heartbeat(timeout=5)
#         master.target_system = master.target_system or 1
#         master.target_component = master.target_component or 1
#         time.sleep(0.3)

#         msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
#         if not msg:
#             print('[ERROR] No HEARTBEAT received after connect.')
#             return False

#         current_mode = mavutil.mode_string_v10(msg)
#         print(f'[INFO] Current flight mode: {current_mode}')

#         mode_map = master.mode_mapping()
#         if mode not in mode_map:
#             print(f'[WARNING] Mode "{mode}" not in mode map.')
#             return False

#         print(f'[INFO] Switching to {mode} …')
#         base_mode, main_mode, sub_mode = mode_map[mode]
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             0,
#             base_mode, main_mode, sub_mode,
#             0, 0, 0, 0
#         )

#         # Wait briefly for ACK (best-effort)
#         t_end = time.time() + 1.5
#         ack_ok = False
#         while time.time() < t_end:
#             ack = master.recv_match(type='COMMAND_ACK', blocking=False)
#             if ack and getattr(ack, 'command', None) == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
#                 print(f'[INFO] Mode ACK: result={ack.result}')
#                 ack_ok = (ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
#                 break
#             time.sleep(0.05)

#         # Fire-and-forget rotation so we don't block the ROS callback
#         def _rotate_once():
#             try:
#                 yaw_rad = math.radians(30)
#                 for _ in range(20):  # ~6 seconds total at 10 Hz * 0.3s
#                     timestamp_ms = int((time.time() * 1000) % 4294967295)
#                     master.mav.set_position_target_local_ned_send(
#                         timestamp_ms,
#                         master.target_system,
#                         master.target_component,
#                         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#                         0b0000011111000111,  # hold vx, vy, vz; set yaw
#                         0, 0, 0,              # pos ignored
#                         0, 0, 0,              # vx, vy, vz = 0
#                         0, 0, 0,              # accel ignored
#                         yaw_rad,              # yaw (absolute)
#                         0.0                   # yaw rate
#                     )
#                     time.sleep(0.3)
#                 print('[INFO] Rotation completed.')
#             except Exception as e:
#                 print(f'[WARNING] Rotation command failed: {e}')

#         threading.Thread(target=_rotate_once, daemon=True).start()

#         return ack_ok or True  # be tolerant if ACK not seen
#     except Exception as e:
#         print(f'[ERROR] Failed to set mode: {e}')
#         return False





















# from pymavlink import mavutil #-- karan code
# import time
# import math

# def set_flight_mode(mode):
#     try:
#         # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
#         master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
#         master.wait_heartbeat(timeout=5)
#         master.target_system = master.target_system or 1
#         master.target_component = master.target_component or 1
#         time.sleep(0.5)
        
#         msg = master.recv_match(type='HEARTBEAT', blocking=True)
        
#         base = msg.base_mode
#         custom = msg.custom_mode
#         sub = msg.system_status

#         current_mode = master.flightmode
#         # current_mode = mavutil.mode_string_v10(msg)
#         print(f'[INFO] Current flight mode: {current_mode}')

#         mode_map = master.mode_mapping()

#         if current_mode not in mode_map:
#             print(f'[WARNING] Mode "{current_mode}" not in mode map.')

#         else:
#             print(f'[INFO] Switching to {mode} mode.')
#             base_mode, main_mode, sub_mode = mode_map[mode]
#             master.mav.command_long_send(
#                 master.target_system,
#                 master.target_component,
#                 mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#                 0,
#                 base_mode, main_mode, sub_mode,
#                 0, 0, 0, 0
#             ) 
#             print('[INFO] Sent set mode command.')

#             # # 2. Send velocity=0 and yaw=+30°
#             # yaw_deg = 30
#             # # yaw_rad = math.radians(yaw_deg)
#             # yaw_rate = math.radians(15)  # rotate right at 15 deg/s

#             # print('[INFO] Holding position and rotating 30°...')
#             # try:
#             #     yaw_rad = math.radians(30)
#             #     for _ in range(20):  # 2 seconds at 10 Hz
#             #         timestamp_ms = int((time.time() * 1000) % 4294967295)
#             #         master.mav.set_position_target_local_ned_send(
#             #             timestamp_ms,
#             #             master.target_system,
#             #             master.target_component,
#             #             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#             #             0b0000011111000111,  # Bitmask: control vx, vy, vz, yaw
#             #             0, 0, 0,              # Position (ignored)
#             #             0, 0, 0,              # vx, vy, vz = hold
#             #             0, 0, 0,              # accel (ignored)
#             #             yaw_rad,              # yaw (absolute)
#             #             0.0                   # yaw rate
#             #         )
#             #         time.sleep(0.3)

#             # except Exception as e:
#             #     print(f'[WARNING] Rotation command failed: {e}')

#             # time.sleep(3)
                
#             # print(f'[INFO] Reverting to {current_mode} mode.')
#             # master.mav.command_long_send(
#             #     master.target_system,
#             #     master.target_component,
#             #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#             #     0,
#             #     base, custom, sub,
#             #     0, 0, 0, 0
#             # )
#             # print('[INFO] Sent set mode command.')
#             return True

#         # Handle regular mode change
#         # if mode not in mode_map:
#         #     print(f'[ERROR] Mode "{mode}" not supported.')
#         #     return False

#         # base_mode, main_mode, sub_mode = mode_map[mode]
#         # master.mav.command_long_send(
#         #     master.target_system,
#         #     master.target_component,
#         #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#         #     0,
#         #     base_mode, main_mode, sub_mode,
#         #     0, 0, 0, 0
#         # )

#         # print('Working')
#         # print(f'[INFO] Switched from {current_mode} → {mode}')
#         # return True

#     except Exception as e:
#         print(f'[ERROR] Failed to set mode: {e}')
#         return False


