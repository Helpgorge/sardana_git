#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading
import re
from sardana import State
from sardana.pool.controller import CounterTimerController, DataAccess, DefaultValue

ReadOnly = DataAccess.ReadOnly
ReadWrite = DataAccess.ReadWrite


class DCS210Controller(CounterTimerController):
    """
    Контроллер счётчика фотонов DCS210 (ZOLIX) для Sardana.
    Первая добавленная ось - счётчик фотонов.
    Вторая добавленная ось - таймер, если он нужен в конфигурации Sardana.
    """

    default_timer = 1
    default_integration_time = 1.0

    ctrl_properties = {
        'SerialPort': {
            'type': str,
            'description': 'Serial port',
            DefaultValue: '/dev/ttyUSB0'
        },
        'BaudRate': {
            'type': int,
            'description': 'Baud rate',
            DefaultValue: 9600
        },
        'Timeout': {
            'type': float,
            'description': 'Serial timeout (seconds)',
            DefaultValue: 2.0
        },
        'CounterAxis': {
            'type': int,
            'description': 'Counter axis. Use -1 to assign the first added axis automatically.',
            DefaultValue: -1
        },
        'TimerAxis': {
            'type': int,
            'description': 'Timer axis. Use -1 to assign the second added axis automatically.',
            DefaultValue: -1
        }
    }

    axis_attributes = {
        'Offset': {'type': float, 'access': ReadWrite},
        'Timer': {'type': bool, 'access': ReadOnly},
    }

    def __init__(self, inst, props, *args, **kwargs):
        super().__init__(inst, props, *args, **kwargs)

        self._serial = None
        self._lock = threading.RLock()
        self._is_acquiring = False
        self._acquisition_thread = None
        self._acquisition_error = None
        self._current_time = self.default_integration_time
        self._last_value = 0
        self._offsets = {}
        self._devices = set()
        self._counter_axis = self.CounterAxis if self.CounterAxis >= 0 else None
        self._timer_axis = self.TimerAxis if self.TimerAxis >= 0 else None
        self._line_terminator = "\r\n"

        port = self.SerialPort
        baud = self.BaudRate
        timeout = self.Timeout
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=timeout,
                write_timeout=timeout
            )
            self._log.info("Serial port %s opened at %d baud", port, baud)
        except Exception as e:
            self._log.error("Cannot open port %s: %s", port, e)
            raise

        self._init_device()

    def _init_device(self):
        """Отправка команд инициализации."""
        self._handshake()
        self._send_command("DAQ_MODE Q")
        self._send_command("COUNT_MODE 3")
        self._send_command("COUNT_SAMPLINGTIME 1000000")
        self._send_command("COUNT_DWELLTIME 0")
        self._log.info("DCS210 initialized")

    def delete_device(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._log.info("Serial port closed")

    def _handshake(self):
        """Establish communication and detect the line terminator accepted by DCS210."""
        errors = []
        for terminator in ("\r\n", "\r", "\n"):
            for command in ("Hello", "HELLO", "hello"):
                try:
                    self._send_command(command, timeout=self.Timeout, terminator=terminator)
                    self._line_terminator = terminator
                    self._log.info(
                        "DCS210 handshake succeeded with terminator %r",
                        terminator
                    )
                    return
                except Exception as exc:
                    errors.append("%s/%r: %s" % (command, terminator, exc))
        raise RuntimeError("DCS210 handshake failed: " + "; ".join(errors))

    def _send_command(self, command, timeout=None, terminator=None):
        """Отправка команды и получение ответа."""
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Serial port not available")
        if timeout is None:
            timeout = self.Timeout
        if terminator is None:
            terminator = self._line_terminator

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write((command + terminator).encode())
            self._serial.flush()
            self._log.debug("Sent: %s", command)

            lines = []
            line = ""
            start_wait = time.time()
            while time.time() - start_wait < timeout:
                raw = self._serial.read(1)
                if not raw:
                    continue
                char = raw.decode(errors='ignore')
                if char in "\r\n":
                    text = line.strip()
                    line = ""
                    if not text:
                        continue
                    lines.append(text)
                    if text == "OK" or re.match(r"^E\d\d$", text):
                        break
                    continue
                line += char

            if line.strip():
                lines.append(line.strip())

            if not lines:
                raise RuntimeError("Timeout waiting response for %s" % command)
            if re.match(r"^E\d\d$", lines[-1]):
                raise RuntimeError("DCS210 returned %s for %s" % (lines[-1], command))

            response = "\n".join(lines)
            self._log.debug("Response: %s", response)
            return response

    def _read_counter_value(self):
        """Чтение значения счётчика (для оси 0)."""
        resp = self._send_command("DATA_COUNT?", timeout=self._current_time + self.Timeout)
        if not resp:
            raise RuntimeError("Empty DATA_COUNT? response")

        data_line = None
        for line in resp.splitlines():
            if line == "OK" or line == "DATA_COUNT?":
                continue
            if line.startswith("DATA_COUNT") or re.search(r"[-+]?\d", line):
                data_line = line
                break
        if data_line is None:
            raise RuntimeError("Cannot find DATA_COUNT value in: %s" % resp)

        matches = re.findall(r"[-+]?\d+(?:\.\d+)?", data_line)
        if not matches:
            raise RuntimeError("Cannot parse value from: %s" % resp)
        return int(float(matches[-1]))

    def _set_samplingtime(self, seconds):
        """Установка времени интегрирования (для оси 0)."""
        seconds = float(seconds)
        if seconds <= 0:
            raise ValueError("Integration time must be positive")

        microseconds = int(round(seconds * 1000000))
        if microseconds < 1:
            microseconds = 1
        if microseconds > 100000000:
            raise ValueError("Integration time must be <= 100 s")
        self._send_command(f"COUNT_SAMPLINGTIME {microseconds}")
        self._current_time = microseconds / 1000000.0
        self._log.debug("Samplingtime set to %d us", microseconds)

    def _acquire(self):
        try:
            self._acquisition_error = None
            self._last_value = self._read_counter_value()
        except Exception as e:
            self._acquisition_error = e
            self._log.error("Acquisition error: %s", e)
        finally:
            self._is_acquiring = False

    def _stop_acquisition(self):
        self._is_acquiring = False
        if not self._serial or not self._serial.is_open:
            return
        try:
            self._serial.write(("Stop" + self._line_terminator).encode())
            self._serial.flush()
        except Exception as e:
            self._log.warning("Stop command failed: %s", e)

    def _assign_axis_role(self, axis):
        if axis == self._counter_axis or axis == self._timer_axis:
            return
        if self._counter_axis is None:
            self._counter_axis = axis
            return
        if self._timer_axis is None:
            self._timer_axis = axis
            return
        raise RuntimeError(
            "Unsupported axis %s. DCS210Controller supports one counter axis "
            "and one optional timer axis." % axis
        )

    def _axis_role(self, axis):
        if axis == self._counter_axis:
            return "counter"
        if axis == self._timer_axis:
            return "timer"
        raise RuntimeError(f"Unsupported axis {axis}")

    # ---------- Обязательные методы таймера ----------
    def get_default_timer(self):
        if self._timer_axis is not None:
            return self._timer_axis
        if self._counter_axis is not None:
            return self._counter_axis
        return self.default_timer

    def get_timer_par(self, axis):
        return self._current_time

    def set_timer_par(self, axis, value):
        self._set_samplingtime(float(value))

    # ---------- Sardana API ----------
    def AddDevice(self, axis):
        self._assign_axis_role(axis)
        self._devices.add(axis)
        self._offsets.setdefault(axis, 0.0)
        self._log.debug("AddDevice for axis %d as %s", axis, self._axis_role(axis))

    def DeleteDevice(self, axis):
        self._devices.discard(axis)
        self._offsets.pop(axis, None)
        if axis == self._counter_axis:
            self._counter_axis = None
        if axis == self._timer_axis:
            self._timer_axis = None
        self._log.debug("DeleteDevice for axis %d", axis)

    def StateOne(self, axis):
        if not self._serial or not self._serial.is_open:
            return State.Fault, "Serial port not open"
        if self._acquisition_error is not None:
            return State.Fault, str(self._acquisition_error)
        if self._is_acquiring:
            return State.Moving, "Acquiring"
        return State.On, "Ready"

    def ReadOne(self, axis):
        """
        Чтение значения:
        - для оси-счётчика возвращает количество фотонов
        - для оси-таймера возвращает установленное время интегрирования
        """
        role = self._axis_role(axis)
        if role == "counter":
            return self._last_value + self._offsets.get(axis, 0.0)
        if role == "timer":
            # Возвращаем текущее установленное время
            return float(self._current_time)
        raise RuntimeError(f"Unsupported axis {axis}")

    def LoadOne(self, axis, value, repetitions=1, latency=0):
        """
        Установка времени интегрирования.
        Для оси-таймера команда игнорируется.
        """
        role = self._axis_role(axis)
        if role == "counter":
            seconds = float(value)
            self._set_samplingtime(seconds)
            self._log.info("LoadOne: timer set to %.3f s", seconds)

    def StartOne(self, axis, value=None):
        role = self._axis_role(axis)
        if role == "timer":
            return
        if self._is_acquiring:
            raise RuntimeError("Acquisition already running")
        if self._acquisition_thread and self._acquisition_thread.is_alive():
            raise RuntimeError("Previous acquisition is still finishing")

        if value is not None:
            self._set_samplingtime(float(value))

        self._acquisition_error = None
        self._is_acquiring = True
        self._log.debug("StartOne for axis %d", axis)
        self._acquisition_thread = threading.Thread(target=self._acquire, daemon=True)
        self._acquisition_thread.start()

    def StopOne(self, axis):
        if self._axis_role(axis) == "counter":
            self._stop_acquisition()
        self._log.debug("StopOne for axis %d", axis)

    def AbortOne(self, axis):
        if self._axis_role(axis) == "counter":
            self._stop_acquisition()
        self._log.debug("AbortOne for axis %d", axis)

    def PreStartOne(self, axis, value):
        if self._axis_role(axis) == "counter" and value is not None:
            self._set_samplingtime(float(value))
        return True

    def GetExtraAttributePar(self, axis, name):
        if name == "Offset":
            return self._offsets.get(axis, 0.0)
        elif name == "Timer":
            return self._axis_role(axis) == "counter"
        return None

    def SetExtraAttributePar(self, axis, name, value):
        if name == "Offset":
            self._offsets[axis] = float(value)

    def Close(self):
        self._stop_acquisition()
        if self._acquisition_thread and self._acquisition_thread.is_alive():
            self._acquisition_thread.join(timeout=float(self.Timeout))
        self.delete_device()
