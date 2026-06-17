#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import threading
from sardana import State
from sardana.pool.controller import CounterTimerController, DataAccess

ReadOnly = DataAccess.ReadOnly
ReadWrite = DataAccess.ReadWrite


class DCS210Controller(CounterTimerController):
    """
    Контроллер счётчика фотонов DCS210 (ZOLIX) для Sardana.
    Ось 0 - счётчик фотонов (возвращает количество фотонов).
    Ось 1 - таймер (возвращает установленное время интегрирования).
    """

    default_timer = 1.0

    ctrl_properties = {
        'SerialPort': {
            'type': str,
            'description': 'Serial port',
            'default_value': '/dev/ttyUSB0'
        },
        'BaudRate': {
            'type': int,
            'description': 'Baud rate',
            'default_value': 9600
        },
        'Timeout': {
            'type': float,
            'description': 'Serial timeout (seconds)',
            'default_value': 2.0
        }
    }

    axis_attributes = {
        'Offset': {'type': float, 'access': ReadWrite},
        'Timer': {'type': bool, 'access': ReadOnly},
    }

    def __init__(self, inst, props, *args, **kwargs):
        super().__init__(inst, props, *args, **kwargs)

        self._serial = None
        self._lock = threading.Lock()
        self._is_acquiring = False
        self._current_time = self.default_timer
        self._last_value = 0

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
        self._send_command("Hello")
        self._send_command("DAQ_MODE Q")
        self._send_command("COUNT_MODE 3")
        self._send_command("COUNT_SAMPLINGTIME 1000")
        self._send_command("COUNT_DWELLTIME 0")
        self._log.info("DCS210 initialized")

    def delete_device(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._log.info("Serial port closed")

    def _send_command(self, command, timeout=2.0):
        """Отправка команды и получение ответа."""
        if not self._serial:
            raise RuntimeError("Serial port not available")

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write((command + '\r').encode())
            self._log.debug("Sent: %s", command)

            response = ""
            start_wait = time.time()
            while time.time() - start_wait < timeout:
                if self._serial.in_waiting:
                    char = self._serial.read().decode(errors='ignore')
                    response += char
                    if char == '\r' and not self._serial.in_waiting:
                        break
                    if response.endswith("OK\r"):
                        break
            self._log.debug("Response: %s", response.strip())
            return response.strip() if response else None

    def _read_counter_value(self):
        """Чтение значения счётчика (для оси 0)."""
        resp = self._send_command("DATA_COUNT?", timeout=5.0)
        if resp:
            parts = resp.split()
            if len(parts) > 1:
                try:
                    return int(parts[1])
                except (ValueError, IndexError):
                    self._log.warning("Cannot parse value from: %s", resp)
        return 0

    def _set_samplingtime(self, seconds):
        """Установка времени интегрирования (для оси 0)."""
        ms = int(seconds * 1000)
        if ms < 1:
            ms = 1
        self._send_command(f"COUNT_SAMPLINGTIME {ms}", timeout=1.0)
        self._current_time = seconds
        self._log.debug("Samplingtime set to %d ms", ms)

    # ---------- Обязательные методы таймера ----------
    def get_default_timer(self):
        return self.default_timer

    def get_timer_par(self, axis):
        return self._current_time

    def set_timer_par(self, axis, value):
        self._set_samplingtime(float(value))

    # ---------- Sardana API ----------
    def AddDevice(self, axis):
        self._log.debug("AddDevice for axis %d", axis)

    def DeleteDevice(self, axis):
        self._log.debug("DeleteDevice for axis %d", axis)

    def StateOne(self, axis):
        if not self._serial or not self._serial.is_open:
            return State.Fault, "Serial port not open"
        if self._is_acquiring:
            return State.Moving, "Acquiring"
        return State.On, "Ready"

    def ReadOne(self, axis):
        """
        Чтение значения:
        - для оси 0 (счётчик) возвращает количество фотонов (int)
        - для оси 1 (таймер) возвращает установленное время (float)
        """
        if axis == 0:
            try:
                value = self._read_counter_value()
                self._last_value = value
                return value
            except Exception as e:
                self._log.error("ReadOne error for axis 0: %s", e)
                return 0
        elif axis == 1:
            # Возвращаем текущее установленное время
            return float(self._current_time)
        else:
            raise RuntimeError(f"Unsupported axis {axis}")

    def LoadOne(self, axis, value, repetitions=1, latency=0):
        """
        Установка времени интегрирования.
        Только для оси 0 (счётчик) — для оси 1 (таймер) игнорируем.
        """
        if axis == 0:
            seconds = float(value)
            self._set_samplingtime(seconds)
            self._log.info("LoadOne: timer set to %.3f s", seconds)
        # Для оси 1 ничего не делаем

    def StartOne(self, axis, value=None):
        self._is_acquiring = True
        self._log.debug("StartOne for axis %d", axis)
        if axis == 0:
            try:
                self._last_value = self._read_counter_value()
            except Exception as e:
                self._log.error("Read error in StartOne: %s", e)
                self._last_value = 0
        self._is_acquiring = False

    def StopOne(self, axis):
        self._is_acquiring = False
        self._log.debug("StopOne for axis %d", axis)

    def AbortOne(self, axis):
        self._is_acquiring = False
        self._log.debug("AbortOne for axis %d", axis)

    def PreStartOne(self, axis, value):
        return True

    def GetExtraAttributePar(self, axis, name):
        if name == "Offset":
            return 0.0
        elif name == "Timer":
            return 1   # 1 = True (этот канал поддерживает таймер)
        return None

    def SetExtraAttributePar(self, axis, name, value):
        if name == "Offset":
            pass

    def Close(self):
        self.delete_device()