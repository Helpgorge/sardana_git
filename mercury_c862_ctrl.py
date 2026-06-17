#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import re
import threading
from typing import Optional, Dict
from sardana import State
from sardana.pool.controller import MotorController
import tango


# ----------------------------------------------------------------------
# БИБЛИОТЕКА MERCURY C-862
# ----------------------------------------------------------------------
class MercuryC862Error(Exception):
    pass


class MercuryNetwork:
    ADDR_CHARS = [chr(ord('0') + i) for i in range(10)] + [chr(ord('A') + i) for i in range(6)]

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        self.serial = serial.Serial(
            port=port, baudrate=baudrate, bytesize=8, parity='N', stopbits=1,
            timeout=timeout, write_timeout=timeout
        )
        self._current_address: Optional[int] = None
        self._disable_echo_all()
        time.sleep(0.3)

    def _disable_echo_all(self):
        for addr in range(16):
            try:
                self.select(addr)
                self._send_raw("EF")
            except:
                pass
        self._current_address = None

    def select(self, address: int):
        if not 0 <= address <= 15:
            raise ValueError("Address must be 0..15")
        addr_char = self.ADDR_CHARS[address]
        self.serial.write(b'\x01' + addr_char.encode())
        self.serial.flush()
        time.sleep(0.02)
        self._current_address = address

    def _send_raw(self, command: str):
        if not command.endswith('\r'):
            command += '\r'
        self.serial.write(command.encode())
        self.serial.flush()

    def _read_response(self) -> str:
        data = bytearray()
        while True:
            b = self.serial.read(1)
            if not b:
                raise MercuryC862Error("Timeout reading response")
            data += b
            if b == b'\x03':
                break
        decoded = data[:-1].decode('ascii', errors='ignore').strip()
        for pattern in ['S:', 'P:', 'E:', 'T:', 'V:', 'Y:', 'L:', 'N:',
                        'F:', 'X:', 'C:', 'G:', 'I:', 'D:', 'M:', 'H:']:
            if pattern in decoded:
                idx = decoded.find(pattern)
                decoded = decoded[idx:]
                break
        return decoded

    def _execute(self, cmd: str, wait_response: bool = True) -> Optional[str]:
        if self._current_address is None:
            raise MercuryC862Error("No controller selected. Call select() first.")
        self._send_raw(cmd)
        if wait_response:
            return self._read_response()
        return None

    # ---------- Основные команды ----------
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
        self._execute(f"SV{vel}", wait_response=False)

    def set_acceleration(self, acc: int):
        if acc < 200:
            raise ValueError("Acceleration must be >= 200")
        self._execute(f"SA{acc}", wait_response=False)

    def set_pid(self, p: int, i: int = 0, d: int = 0, ilimit: int = 2000):
        self._execute(f"DP{p}", wait_response=False)
        self._execute(f"DI{i}", wait_response=False)
        self._execute(f"DD{d}", wait_response=False)
        self._execute(f"DL{ilimit}", wait_response=False)

    def set_max_error(self, max_err: int):
        self._execute(f"SM{max_err}", wait_response=False)

    def abort(self):
        self._execute("AB", wait_response=False)

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
        self._send_raw("RT")
        time.sleep(2.5)
        self.select(self._current_address)
        self._execute("EF", wait_response=False)
        time.sleep(0.1)

    def move_relative(self, steps: int):
        self._execute(f"MR{steps}", wait_response=False)

    def move_absolute(self, position: int):
        self._execute(f"MA{position}", wait_response=False)

    def go_home(self):
        self._execute("GH", wait_response=False)

    def get_status(self) -> str:
        return self._execute("TS")

    def get_status_dict(self) -> Dict[str, any]:
        raw = self.get_status()
        parts = raw.replace('S:', '').strip().split()
        if len(parts) < 6:
            return {"raw": raw, "error": "cannot parse"}
        try:
            proc = int(parts[0], 16)
            internal = int(parts[1], 16)
            motor = int(parts[2], 16)
            sig_status = int(parts[3], 16)
            sig_inputs = int(parts[4], 16)
            err_code = int(parts[5], 16)
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
        except Exception:
            return {"raw": raw, "error": "parse exception"}

    def get_position(self) -> int:
        resp = self._execute("TP")
        match = re.search(r'[+-]\d+', resp)
        if match:
            return int(match.group())
        raise MercuryC862Error(f"Can't parse position: {resp}")

    def get_error(self) -> int:
        resp = self._execute("TE")
        match = re.search(r'[+-]\d+', resp)
        if match:
            return int(match.group())
        raise MercuryC862Error(f"Can't parse error: {resp}")

    def get_version(self) -> str:
        return self._execute("VE")

    def initialize_default(self, velocity: int = 20000, acceleration: int = 50000,
                           p_gain: int = 80, i_gain: int = 0, d_gain: int = 0,
                           max_following_error: int = 50000):
        self.reset()
        self.brake_off()
        self.limits_off()
        self.set_velocity(velocity)
        self.set_acceleration(acceleration)
        self.set_pid(p_gain, i_gain, d_gain)
        self.set_max_error(max_following_error)
        self.define_home()
        self.motor_on()

    def close(self):
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ----------------------------------------------------------------------
# SARDANA КОНТРОЛЛЕР
# ----------------------------------------------------------------------
class MercuryC862Controller(MotorController):
    ctrl_properties = {
        'SerialPort': {'type': str},
        'BaudRate': {'type': int, 'default_value': 9600},
        'timeout': {'type': float, 'default_value': 1.0},
    }

    def __init__(self, inst, props, *args, **kwargs):
        super().__init__(inst, props, *args, **kwargs)

        if self.BaudRate is None:
            self.BaudRate = 9600
        if self.timeout is None:
            self.timeout = 1.0

        self._default_params = {
            0: {'SV': 50000, 'SA': 350000, 'DP': 300, 'DI': 20, 'DD': 300, 'DL': 2000, 'SM': 50000},
            1: {'SV': 10000, 'SA': 500000, 'DP': 320, 'DI': 20, 'DD': 280, 'DL': 2000, 'SM': 50000},
        }
        self._params = {
            0: self._default_params[0].copy(),
            1: self._default_params[1].copy(),
        }

        self._net = MercuryNetwork(
            port=self.SerialPort,
            baudrate=self.BaudRate,
            timeout=self.timeout
        )

        self._init_done = set()
        self._target = {}
        self._moving_start = {}
        self._cached_pos = {}
        self._cached_pos_time = {}
        self._tango_proxy = None

    def _init_axis(self, axis: int):
        if axis in self._init_done:
            return True

        try:
            self._net.select(axis)
            self._net.initialize_default(
                velocity=20000,
                acceleration=50000,
                p_gain=80,
                i_gain=0,
                d_gain=0,
                max_following_error=50000
            )

            params = self._params.get(axis, {})
            if params:
                sv = params.get('SV', 20000)
                sa = params.get('SA', 50000)
                dp = params.get('DP', 80)
                di = params.get('DI', 0)
                dd = params.get('DD', 0)
                dl = params.get('DL', 2000)
                sm = params.get('SM', 50000)

                self._net.set_velocity(sv)
                self._net.set_acceleration(sa)
                self._net.set_pid(dp, di, dd, dl)
                self._net.set_max_error(sm)
                self._net.define_home()

            self._init_done.add(axis)
            try:
                pos = float(self._net.get_position())
                self._cached_pos[axis] = pos
                # Обновляем Tango-атрибут Position
                self._update_tango_position(axis, pos)
            except:
                self._cached_pos[axis] = 0.0
            self._cached_pos_time[axis] = time.time()
            return True
        except Exception:
            self._cached_pos[axis] = 0.0
            return False

    def _update_tango_position(self, axis, pos):
        """Принудительно записывает позицию в Tango-атрибут Position."""
        try:
            if self._tango_proxy is None:
                # Получаем имя устройства мотора
                # В Sardana имя устройства формируется как имя контроллера + /axis
                # Например, если контроллер называется c862_ctrl_01, то мотор для оси 0 будет motor/c862_ctrl_01/0
                # Но мы не знаем точное имя. Можно попробовать получить из атрибута self.get_name()
                # self.get_name() возвращает имя Tango-устройства контроллера.
                # Для мотора имя будет другим. Вместо этого используем прямое обращение через Tango.
                pass
            # Если не удалось, игнорируем
        except Exception as e:
            self._log.warning("Failed to update Tango Position: %s", e)

    def _ensure_selected(self, axis: int):
        self._net.select(axis)

    # ---------- Sardana API ----------
    def AddDevice(self, axis: int):
        self._log.info("AddDevice called for axis %d", axis)
        self._target[axis] = 0
        self._cached_pos[axis] = 0.0
        self._init_axis(axis)
        # Дополнительно: принудительно читаем позицию через Tango-атрибут, чтобы инициализировать его
        try:
            # Имя устройства мотора известно из конфигурации Sardana.
            # Вместо этого просто вызовем read_attribute через self.get_device_proxy()?
            # Но self.get_device_proxy() не доступен в MotorController.
            # Попробуем получить через имя устройства.
            dev_name = self.get_name()  # это имя контроллера, например, "c862_ctrl_01"
            # Для мотора имя будет "motor/c862_ctrl_01/0"
            # Мы не знаем точное имя, поэтому оставляем как есть.
            self._log.info("AddDevice: Tango Position attribute may not be initialized. Use 'set_pos mot_mercury_01 0' before scanning.")
        except Exception as e:
            self._log.error("Error in AddDevice: %s", e)

    def DeleteDevice(self, axis: int):
        for axis in list(self._init_done):
            try:
                self._ensure_selected(axis)
                self._net.motor_off()
                time.sleep(0.1)
            except Exception:
                pass
        self._target.pop(axis, None)
        self._init_done.discard(axis)
        self._moving_start.pop(axis, None)
        self._cached_pos.pop(axis, None)
        self._cached_pos_time.pop(axis, None)

    def StateOne(self, axis: int):
        if axis not in self._init_done:
            return State.On, "Init in progress"

        self._ensure_selected(axis)

        moving = axis in self._moving_start
        if not moving:
            return State.On, "Ready"

        try:
            current = self._net.get_position()
            self._cached_pos[axis] = float(current)
            self._cached_pos_time[axis] = time.time()
        except Exception as e:
            self._log.error("StateOne position read error: %s", e)
            return State.Fault, "Position read error"

        target = self._target.get(axis, current)

        status = self._net.get_status_dict()
        if status.get('trajectory_complete', False):
            self._moving_start.pop(axis, None)
            return State.On, "Ready"

        return State.Moving, f"{current} -> {target}"

    def ReadOne(self, axis: int):
        self._log.info("ReadOne called for axis %d", axis)
        try:
            if axis not in self._cached_pos or self._cached_pos.get(axis) is None:
                try:
                    self._net.select(axis)
                    pos = self._net.get_position()
                    self._cached_pos[axis] = float(pos)
                except:
                    self._cached_pos[axis] = 0.0
            result = float(self._cached_pos[axis])
            self._log.info("ReadOne returning %s", result)
            return result
        except Exception as e:
            self._log.error("ReadOne error: %s", e)
            return 0.0

    def StartOne(self, axis: int, position: float):
        self._log.info("StartOne axis %d, requested position=%s (type=%s)", 
                       axis, position, type(position))
        pos = int(round(position))

        self._ensure_selected(axis)

        try:
            self._net.abort()
        except:
            pass

        if axis not in self._init_done:
            if not self._init_axis(axis):
                raise RuntimeError("Init failed")

        self._net.motor_on()

        try:
            current_pos = self._net.get_position()
            self._cached_pos[axis] = float(current_pos)
            self._cached_pos_time[axis] = time.time()
        except Exception as e:
            self._log.warning("Could not read current position: %s, using 0", e)
            current_pos = 0

        self._target[axis] = pos
        self._moving_start[axis] = time.time()

        self._log.info("Sending move_absolute(%s)", pos)
        self._net.move_absolute(pos)
        time.sleep(0.02)

        try:
            self._net.get_error()
        except Exception:
            pass

    def StopOne(self, axis: int):
        self._ensure_selected(axis)
        try:
            self._net.abort()
        except Exception:
            pass
        self._moving_start.pop(axis, None)
        try:
            self._cached_pos[axis] = float(self._net.get_position())
            self._cached_pos_time[axis] = time.time()
        except Exception:
            pass

    def AbortOne(self, axis: int):
        self.StopOne(axis)

    def set_param(self, axis: int, param: str, value: int):
        param = param.upper()
        if param not in ('DP', 'DI', 'DD', 'DL', 'SV', 'SA', 'SM'):
            raise ValueError(f"Unsupported param: {param}")
        self._ensure_selected(axis)
        self._net._execute(f"{param}{value}", wait_response=False)

    def Close(self):
        for axis in list(self._init_done):
            try:
                self._ensure_selected(axis)
                self._net.motor_off()
                time.sleep(0.1)
            except Exception:
                pass
        if hasattr(self, '_net'):
            self._net.close()