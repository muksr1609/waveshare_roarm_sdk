# coding=utf-8

from __future__ import division
import time
import threading
import serial
import requests
import json

from roarm_sdk.generate import CommandGenerator
from roarm_sdk.common import JsonCmd, BaseController, write, read
from roarm_sdk.utils import calibration_parameters


class roarm(CommandGenerator):
    """
    Roarm Python API communication class.
    """
    def __init__(self, roarm_type=None, port=None, baudrate=115200, host=None, timeout=0.1, debug=False, thread_lock=True, gripper_type="angular_direct"):
        """
        Args:
            roarm_type    : port string
            port          : port string
            baudrate      : baud rate string, default '115200'
            host          : host string. HTTP mode is control-only (send cmds);
                            feedback / drag-teach recording need serial.
            timeout       : default 0.1
            debug         : whether show debug info
            gripper_type  : "angular_direct" or "angular_gear", default "angular_direct"
        """
        self.type = roarm_type
        super(roarm, self).__init__(self.type, debug, gripper_type)
        self.calibration_parameters = calibration_parameters
        self.thread_lock = thread_lock
        self.host =None            
        self.stop_flag = False
        self.base_controller = None
        self._torque_recovering = False
        self._torque_recover_timeout = 30.0

        if thread_lock:
            self.lock = threading.Lock()
        if host:
            self.host = host
        else:    
            self._serial_port = serial.Serial()
            self._serial_port.port = port
            self._serial_port.baudrate = baudrate
            self._serial_port.timeout = timeout
            self._serial_port.rts = False
            self._serial_port.open() 

    _write = write
    _read = read

    def _clear_serial_input(self):
        if self.host:
            return
        try:
            self._serial_port.reset_input_buffer()
            if self.base_controller is not None:
                self.base_controller.rl.clear_buffer()
        except Exception:
            pass

    def _mesg(self, genre, *args):
        """
        Args:
            genre: command type (Command)
            *args: other data.
                   It is converted to octal by default.
                   If the data needs to be encapsulated into hexadecimal,
                   the array is used to include them. (Data cannot be nested)
        """
        real_command = super(roarm, self)._mesg(genre, *args)
        if self.thread_lock:
            with self.lock:
                result = self._res(real_command, genre)
        else:
            result = self._res(real_command, genre)
        # After torque disable, serial feedback may be empty for a while.
        if genre == JsonCmd.TORQUE_SET and args:
            if args[0] == 0:
                self._torque_recovering = True
                self._clear_serial_input()
            elif args[0] == 1:
                self._torque_recovering = False
        return result

    def _set_serial_read_timeout(self, value):
        if self.host:
            return None, None
        old_ser = self._serial_port.timeout
        old_rl = None
        self._serial_port.timeout = value
        if self.base_controller is not None:
            old_rl = self.base_controller.rl.timeout
            self.base_controller.rl.timeout = value
        return old_ser, old_rl

    def _restore_serial_read_timeout(self, old_ser, old_rl):
        if self.host or old_ser is None:
            return
        self._serial_port.timeout = old_ser
        if self.base_controller is not None and old_rl is not None:
            self.base_controller.rl.timeout = old_rl

    def _http_get(self, url, timeout=1.0):
        """GET robot HTTP API without system proxy interference."""
        session = requests.Session()
        session.trust_env = False
        return session.get(url, timeout=timeout)

    def _request_once(self, real_command, genre):
        """Send one request and return raw response data, or None on failure."""
        try:
            if self.host:
                # HTTP is control-only: firmware ACKs {"ok":1}; feedback is serial.
                if genre == JsonCmd.FEEDBACK_GET:
                    return None
                url = f"http://{self.host}/js?json={real_command.decode()}"
                self._http_get(url, timeout=1.0)
                return real_command

            recover = genre == JsonCmd.FEEDBACK_GET and self._torque_recovering
            old_ser = old_rl = None
            if recover:
                if self.base_controller is None:
                    self.base_controller = BaseController(
                        port=self._serial_port, roarm_type=self.type
                    )
                old_ser, old_rl = self._set_serial_read_timeout(1.0)
            try:
                self._write(real_command)
                if genre != JsonCmd.FEEDBACK_GET:
                    return real_command
                return self._read(genre)
            finally:
                if recover:
                    self._restore_serial_read_timeout(old_ser, old_rl)
        except Exception:
            return None

    def _res(self, real_command, genre):
        if self.host and genre == JsonCmd.FEEDBACK_GET:
            return -1

        if genre == JsonCmd.FEEDBACK_GET and self._torque_recovering:
            deadline = time.time() + self._torque_recover_timeout
            while time.time() < deadline:
                data = self._request_once(real_command, genre)
                if data is not None and data != b'':
                    try:
                        res = self._process_received(data, genre)
                        if isinstance(res, list) and len(res) == 1:
                            self._torque_recovering = False
                            return res[0]
                    except Exception:
                        pass
                # Serial: avoid flooding the bus while arm is returning.
                time.sleep(0.3)
            return -1

        try_count = 0
        data = None
        while try_count < 10:
            data = self._request_once(real_command, genre)
            if data is not None and data != b'':
                break
            try_count += 1
        else:
            return -1

        try:
            res = self._process_received(data, genre)
        except Exception:
            return -1
        if res is None:
            return None
        elif isinstance(res, list) and len(res) == 1:
            return res[0]

    def joints_radian_ctrl_once(self, radians, speed, acc):
        """Send one validated T=102 group command without automatic retry.

        This effectful variant uses the same validation and encoding as
        ``joints_radian_ctrl``, including the configured gripper convention, but
        invokes ``_request_once`` exactly once. Feedback/read methods retain their
        existing retry behavior.

        Returns the encoded command bytes on a successful write, or ``-1`` when
        the single request fails.
        """
        genre = JsonCmd.JOINTS_RADIAN_CTRL
        self.calibration_parameters(
            roarm_type=self.type,
            gripper_type=self.gripper_type,
            radians=radians,
            speed=speed,
            acc=acc,
        )
        real_command = super(roarm, self)._mesg(genre, radians, speed, acc)
        if self.thread_lock:
            with self.lock:
                result = self._request_once(real_command, genre)
        else:
            result = self._request_once(real_command, genre)
        if result is None or result == b'':
            return -1
        return result
            
    def breath_led(self, duration=1.0, steps=10):
        """Set breath_led
        Args:
            duration: breath duration, type: float
            steps: breath steps, type: int
        """
        for i in range(steps + 1):
            led = int((i / steps) * 255)  
            self.led_ctrl(led=led)
            time.sleep(duration / (2 * steps))  

        for i in range(steps + 1):
            led = int((1 - i / steps) * 255)  
            self.led_ctrl(led=led)
            time.sleep(duration / (2 * steps)) 
        return 1 
        
    def listen_for_input(self):
        input("Press any to stop data collection...\n")
        self.stop_flag = True
        
    def drag_teach_start(self, filename, sample_hz=10):
        """Start drag teach
        Args:
            filename: file to save data, type: str
            sample_hz: target sample rate (default 10). Serial only.
        """
        if self.host:
            print("drag_teach_start needs serial; HTTP mode is control-only.")
            return
        self.torque_set(cmd=0)
        data = []
        print(f"Starting data collection.")
        input_thread = threading.Thread(target=self.listen_for_input)
        input_thread.daemon = True 
        input_thread.start()
        print("Press any to stop data collection...\n")
        interval = 1.0 / float(sample_hz) if sample_hz and sample_hz > 0 else 0.1
        while not self.stop_flag:
            t0 = time.time()
            radians = self.joints_radian_get()
            if not isinstance(radians, list):
                continue
            record = {
                "timestamped": time.time(),
                "radians": radians
            }
            data.append(record)
            sleep_left = interval - (time.time() - t0)
            if sleep_left > 0:
                time.sleep(sleep_left)
        try:
            with open(filename, "w") as file:
                json.dump(data, file, indent=4)
            print(f"Data saved. Total {len(data)} records.")
        except Exception as e:
            print(f"Error saving data: {e}")

    def drag_teach_replay(self, filename):
        """Replay drag teach data 
        Args:
            filename: file to save data, type: str
        """
        try:
            with open(filename, "r") as file:
                data = json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            print("Error: File not found or empty. Ensure data exists in the file.")
            return

        total_steps = len(data)

        if total_steps < 2:
            print("Error: Not enough data to calculate velocity and acceleration.")
            return
        switch_dict = {
        "roarm_m2": [0, 0, 0, 0], 
        "roarm_m3": [0, 0, 0, 0, 0, 0],
        }
        prev_speed = switch_dict[self.type]  

        for i in range(1, total_steps):
            record1 = data[i-1]
            record2 = data[i]

            timestamp1, radians1 = record1["timestamped"], record1["radians"]
            timestamp2, radians2 = record2["timestamped"], record2["radians"]

            time_diff = timestamp2 - timestamp1
            if time_diff < 0.001:
                print(f"Warning: Tiny time difference at step {i}, skipping.")
                continue

            radians_diff = [r2 - r1 for r1, r2 in zip(radians1, radians2)]    
            angular_velocity = [r_diff / time_diff for r_diff in radians_diff]
            speed = [
                min(4096, abs(int(angular_vel * 2048 / 3.1415926)))
                for angular_vel in angular_velocity
            ]
            acceleration = [
                min(254, abs(int((spd - prev_spd) / (100 * time_diff))))
                for spd, prev_spd in zip(speed, prev_speed)
            ]
            
            for joint, radian, spd, acc in zip(range(1, len(prev_speed)+1), radians2, speed, acceleration):
                if spd != 0:
                    print(f"Speed for joint {joint}: {spd}, Acceleration: {acc}")
                    self.joint_radian_ctrl(joint=joint, radian=radian, speed=spd, acc=acc)
                    time.sleep(time_diff)
            prev_speed = speed

        print(f"Replayed {total_steps} steps from {filename}.")

    def disconnect(self):
        """Disconnect from the roarm 
        """
        if self.host:
            self.host = None
        elif hasattr(self, "_serial_port") and self._serial_port is not None:
            if self._serial_port.is_open:
                self._serial_port.close()
