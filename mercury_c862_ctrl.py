#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import threading
import time
from typing import Dict, Optional

import serial
from sardana import State
from sardana.pool.controller import MotorController


class MercuryC862Error(Exception):
    pass


class MercuryNetwork:
    ADDR_CHARS = [chr(ord("0") + i) for i in range(10)] + [
        chr(ord("A") + i) for i in range(6)
    ]

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
            write_timeout=timeout,
        )
        self._current_address: Optional[int] = None
        self._disable_echo_all()
        time.sleep(0.3)

    def _disable_echo_all(self):
        for addr in range(16):
            try:
                self.select(addr)
                self._send_raw("EF")
            except Exception:
                pass
        self._current_address = None

    def select(self, address: int):
        if not 0 <= address <= 15:
            raise ValueError("Address must be 0..15")
        addr_char = self.ADDR_CHARS[address]
        self.serial.write(b"\x01" + addr_char.encode())
        self.serial.flush()
        time.sleep(0.02)
        self._current_address = address

    def _send_raw(self, command: str):
        if not command.endswith("\r"):
            command += "\r"
        self.serial.write(command.encode())
        self.serial.flush()

    def _read_response(self) -> str:
        data = bytearray()
        while True:
            b = self.serial.read(1)
            if not b:
                raise MercuryC862Error("Timeout reading response")
            data += b
            if b == b"\x03":
                break

        decoded = data[:-1].decode("ascii", errors="ignore").strip()
        for pattern in (
            "S:",
            "P:",
            "E:",
            "T:",
            "V:",
            "Y:",
            "L:",
            "N:",
            "F:",
            "X:",
            "C:",
            "G:",
            "I:",
            "D:",
            "M:",
            "H:",
        ):
            if pattern in decoded:
                decoded = decoded[decoded.find(pattern) :]
                break
        return decoded

    def _execute(self, cmd: str, wait_response: bool = True) -> Optional[str]:
        if self._current_address is None:
            raise MercuryC862Error("No controller selected. Call select() first.")
        self._send_raw(cmd)
        if wait_response:
            return self._read_response()
        return None

    def brake_off(self):
        self._execute("BF", wait_response=False)

    def brake_on(self):
        self._execute("BN", wait_response=False)

    def limits_off(self):
        self._execute("LF", wait_response=False)

    def limits_on(self):
        self._execute("LN", wait_response=False)

    def limits_logic_high(self):
        self._execute("LH", wait_response=False)

    def limits_logic_low(self):
        self._execute("LL", wait_response=False)

    def set_velocity(self, vel: int):
        if not 0 < vel < 500000:
            raise ValueError("Velocity must be 1..499999")
        self._execute(f"SV{int(vel)}", wait_response=False)

    def set_acceleration(self, acc: int):
        if acc < 200:
            raise ValueError("Acceleration must be >= 200")
        self._execute(f"SA{int(acc)}", wait_response=False)

    def set_pid(self, p: int, i: int = 0, d: int = 0, ilimit: int = 2000):
        self._execute(f"DP{int(p)}", wait_response=False)
        self._execute(f"DI{int(i)}", wait_response=False)
        self._execute(f"DD{int(d)}", wait_response=False)
        self._execute(f"DL{int(ilimit)}", wait_response=False)

    def set_max_error(self, max_err: int):
        if not 0 < max_err < 32767:
            raise ValueError("Maximum following error must be 1..32766")
        self._execute(f"SM{int(max_err)}", wait_response=False)

    def abort(self):
        self._execute("AB", wait_response=False)

    def smooth_abort(self):
        self._execute("AB1", wait_response=False)

    def define_home(self):
        self._execute("DH", wait_response=False)

    def motor_on(self):
        self._execute("MN", wait_response=False)
        time.sleep(0.2)

    def motor_off(self):
        self._execute("MF", wait_response=False)

    def reset(self):
        if self._current_address is None:
            raise MercuryC862Error("No controller selected. Call select() first.")
        address = self._current_address
        self._send_raw("RT")
        time.sleep(2.5)
        self.select(address)
        self._execute("EF", wait_response=False)
        time.sleep(0.1)

    def move_absolute(self, position: int):
        self._execute(f"MA{int(position)}", wait_response=False)

    def get_status(self) -> str:
        return self._execute("TS")

    def get_status_dict(self) -> Dict[str, object]:
        raw = self.get_status()
        parts = raw.replace("S:", "").strip().split()
        if len(parts) < 6:
            return {"raw": raw, "error": "cannot parse status"}

        try:
            proc = int(parts[0], 16)
            internal = int(parts[1], 16)
            motor = int(parts[2], 16)
            sig_status = int(parts[3], 16)
            sig_inputs = int(parts[4], 16)
            err_code = int(parts[5], 16)
        except Exception:
            return {"raw": raw, "error": "cannot parse status"}

        return {
            "raw": raw,
            "servo_enabled": not (proc & 0x80),
            "trajectory_complete": bool(proc & 0x04),
            "excessive_error": bool(proc & 0x20),
            "limit_exceeded": bool(proc & 0x10),
            "brake_on": bool(sig_status & 0x08),
            "positive_limit": bool(sig_inputs & 0x04),
            "negative_limit": bool(sig_inputs & 0x08),
            "reference_input": bool(sig_inputs & 0x02),
            "error_code": err_code,
            "internal_flags": internal,
            "motor_flags": motor,
        }

    def get_position(self) -> int:
        resp = self._execute("TP")
        match = re.search(r"[+-]\d+", resp)
        if match:
            return int(match.group())
        raise MercuryC862Error(f"Can't parse position: {resp}")

    def initialize_default(
        self,
        velocity: int = 50000,
        acceleration: int = 350000,
        p_gain: int = 300,
        i_gain: int = 20,
        d_gain: int = 300,
        max_following_error: int = 30000,
    ):
        self.reset()
        self.brake_off()
        self.limits_on()
        self.set_velocity(velocity)
        self.set_acceleration(acceleration)
        self.set_pid(p_gain, i_gain, d_gain)
        self.set_max_error(max_following_error)
        self.define_home()
        self.motor_on()

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()


class MercuryC862Controller(MotorController):
    """
    Sardana motor controller for PI Mercury C-862.

    Mercury address is calculated as:
        address = Sardana axis + AddressOffset

    Use AddressOffset=0 for Sardana axes 0/1 mapped to Mercury addresses 0/1.
    Use AddressOffset=-1 for Sardana axes 1/2 mapped to Mercury addresses 0/1.
    """

    ctrl_properties = {
        "SerialPort": {
            "type": str,
            "description": "Serial port, e.g. /dev/ttyUSB0",
        },
        "BaudRate": {
            "type": int,
            "description": "Serial baud rate",
            "default_value": 9600,
        },
        "Timeout": {
            "type": float,
            "description": "Serial timeout in seconds",
            "default_value": 1.0,
        },
        "AddressOffset": {
            "type": int,
            "description": "Mercury address offset: address = Sardana axis + offset",
            "default_value": 0,
        },
        "InitializeOnAdd": {
            "type": bool,
            "description": "Run Mercury initialization when the motor is added",
            "default_value": True,
        },
        "ResetOnInit": {
            "type": bool,
            "description": "Send RT reset during initialization",
            "default_value": False,
        },
        "DefineHomeOnInit": {
            "type": bool,
            "description": "Define the current hardware position as home during initialization",
            "default_value": False,
        },
        "EnableLimits": {
            "type": bool,
            "description": "Enable Mercury software limit switch handling with LN",
            "default_value": True,
        },
        "LimitActiveHigh": {
            "type": bool,
            "description": "Use LH when true, LL when false",
            "default_value": True,
        },
        "MaxFollowingError": {
            "type": int,
            "description": "Mercury SM value. Valid range is 1..32766.",
            "default_value": 30000,
        },
    }

    _axis_params = {
        0: {
            "DP": 300,
            "DI": 20,
            "DD": 300,
            "DL": 2000,
            "SV": 50000,
            "SA": 350000,
            "ranges": {
                "DP": (100, 350),
                "DI": (0, 50),
                "DD": (0, 400),
                "DL": (0, 2000),
                "SV": (1, 100000),
                "SA": (1000, 450000),
            },
        },
        1: {
            "DP": 320,
            "DI": 20,
            "DD": 280,
            "DL": 2000,
            "SV": 10000,
            "SA": 500000,
            "ranges": {
                "DP": (100, 350),
                "DI": (0, 50),
                "DD": (0, 400),
                "DL": (0, 2000),
                "SV": (1, 180000),
                "SA": (1000, 1000000),
            },
        },
    }

    def __init__(self, inst, props, *args, **kwargs):
        super().__init__(inst, props, *args, **kwargs)
        if not 0 < self.MaxFollowingError < 32767:
            raise ValueError("MaxFollowingError must be in range 1..32766")

        self._lock = threading.RLock()
        self._net = MercuryNetwork(
            port=self.SerialPort,
            baudrate=self.BaudRate,
            timeout=self.Timeout,
        )
        self._init_done = set()
        self._target = {}
        self._cached_pos = {}
        self._last_status = {}

    def _address(self, axis: int) -> int:
        address = int(axis) + int(self.AddressOffset)
        if not 0 <= address <= 15:
            raise ValueError(
                "Mercury address %d is out of range for Sardana axis %d. "
                "Check AddressOffset." % (address, axis)
            )
        return address

    def _select_axis(self, axis: int):
        self._net.select(self._address(axis))

    def _params_for_axis(self, axis: int) -> Dict[str, object]:
        address = self._address(axis)
        params = self._axis_params.get(address)
        if params is None:
            raise ValueError(
                "No motor parameters configured for Mercury address %d "
                "(Sardana axis %d)" % (address, axis)
            )
        return params

    def _check_range(self, params: Dict[str, object], name: str, value: int):
        low, high = params["ranges"][name]
        if not low <= value <= high:
            raise ValueError(
                "%s=%d is out of range for this motor (%d..%d)"
                % (name, value, low, high)
            )

    def _init_axis(self, axis: int):
        if axis in self._init_done:
            return

        with self._lock:
            self._select_axis(axis)
            params = self._params_for_axis(axis)
            if self.ResetOnInit:
                self._net.reset()
            self._net.brake_off()

            if self.LimitActiveHigh:
                self._net.limits_logic_high()
            else:
                self._net.limits_logic_low()
            if self.EnableLimits:
                self._net.limits_on()
            else:
                self._net.limits_off()

            self._net.set_velocity(params["SV"])
            self._net.set_acceleration(params["SA"])
            self._net.set_pid(params["DP"], params["DI"], params["DD"], params["DL"])
            self._net.set_max_error(self.MaxFollowingError)
            if self.DefineHomeOnInit:
                self._net.define_home()
            else:
                self._net.abort()
            self._net.motor_on()
            self._cached_pos[axis] = float(self._net.get_position())
            self._last_status[axis] = self._net.get_status_dict()
            self._init_done.add(axis)

    def _read_position(self, axis: int) -> float:
        with self._lock:
            self._select_axis(axis)
            pos = float(self._net.get_position())
        self._cached_pos[axis] = pos
        return pos

    def _read_status(self, axis: int) -> Dict[str, object]:
        with self._lock:
            self._select_axis(axis)
            status = self._net.get_status_dict()
        self._last_status[axis] = status
        return status

    def AddDevice(self, axis: int):
        self._log.info(
            "Adding Mercury C-862 axis %d at address %d", axis, self._address(axis)
        )
        self._target.pop(axis, None)
        if self.InitializeOnAdd:
            self._init_axis(axis)
        else:
            self._cached_pos[axis] = self._read_position(axis)

    def DeleteDevice(self, axis: int):
        self._log.info("Deleting Mercury C-862 axis %d", axis)
        if axis in self._init_done:
            try:
                with self._lock:
                    self._select_axis(axis)
                    self._net.abort()
                    self._net.motor_off()
            except Exception as exc:
                self._log.warning("Could not stop axis %d while deleting: %s", axis, exc)

        self._target.pop(axis, None)
        self._init_done.discard(axis)
        self._cached_pos.pop(axis, None)
        self._last_status.pop(axis, None)

    def StateOne(self, axis: int):
        if axis not in self._init_done:
            try:
                self._init_axis(axis)
            except Exception as exc:
                self._log.error("Axis %d initialization failed: %s", axis, exc)
                return State.Fault, "Initialization failed: %s" % exc

        try:
            status = self._read_status(axis)
        except Exception as exc:
            self._log.error("StateOne status read error for axis %d: %s", axis, exc)
            return State.Fault, "Status read error: %s" % exc

        if status.get("error"):
            return State.Fault, str(status.get("raw", status["error"]))
        if status.get("error_code"):
            return State.Fault, "Mercury error %s" % status["error_code"]
        if status.get("excessive_error"):
            return State.Fault, "Excessive following error"
        if status.get("limit_exceeded"):
            return State.Alarm, "Limit switch reached"
        if not status.get("servo_enabled", True):
            return State.Off, "Motor is off"

        if status.get("trajectory_complete", False):
            self._target.pop(axis, None)
            return State.On, "Ready"

        target = self._target.get(axis)
        if target is None:
            return State.Moving, "Moving"
        return State.Moving, "Moving to %s" % target

    def ReadOne(self, axis: int):
        try:
            return self._read_position(axis)
        except Exception as exc:
            self._log.error("ReadOne position read error for axis %d: %s", axis, exc)
            raise

    def StartOne(self, axis: int, position: float):
        if axis not in self._init_done:
            self._init_axis(axis)

        pos = int(round(float(position)))
        self._log.info("Moving Mercury C-862 axis %d to %d", axis, pos)

        with self._lock:
            self._select_axis(axis)
            self._net.abort()
            self._net.motor_on()
            self._net.move_absolute(pos)
        self._target[axis] = pos

    def StopOne(self, axis: int):
        with self._lock:
            self._select_axis(axis)
            self._net.smooth_abort()
        try:
            self._cached_pos[axis] = self._read_position(axis)
        except Exception as exc:
            self._log.warning("Could not read stopped position for axis %d: %s", axis, exc)
        self._target.pop(axis, None)

    def AbortOne(self, axis: int):
        with self._lock:
            self._select_axis(axis)
            self._net.abort()
        self._target.pop(axis, None)

    def set_param(self, axis: int, param: str, value: int):
        param = param.upper()
        if param not in ("DP", "DI", "DD", "DL", "SV", "SA", "SM"):
            raise ValueError(f"Unsupported param: {param}")
        value = int(value)
        if param == "SM":
            if not 0 < value < 32767:
                raise ValueError("SM must be in range 1..32766")
        else:
            self._check_range(self._params_for_axis(axis), param, value)
        with self._lock:
            self._select_axis(axis)
            self._net._execute(f"{param}{value}", wait_response=False)

    def Close(self):
        for axis in list(self._init_done):
            try:
                with self._lock:
                    self._select_axis(axis)
                    self._net.motor_off()
                    time.sleep(0.1)
            except Exception as exc:
                self._log.warning("Could not switch off axis %d: %s", axis, exc)
        if hasattr(self, "_net"):
            self._net.close()
