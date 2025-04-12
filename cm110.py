import textwrap
import time
from collections.abc import Mapping
from enum import Enum
from types import MappingProxyType

import serial
import serial.tools.list_ports

# TODO: read/write novram
# TODO: scan status message
# TODO: load device state on connect

LINE_END = b"\x18"  # 24 in decimal


class Cmd(Enum):
    # calibration
    ZERO = 52
    DEC = 1
    INC = 7
    CALIBRATE = 18
    # configure
    ORDER = 51
    QUERY = 56
    SELECT = 26
    SIZE = 55
    SPEED = 13
    UNITS = 50
    ECHO = 27
    # run
    GOTO = 16
    SCAN = 12
    STEP = 54
    RESET = 255


class Query(Enum):
    POSITION = 0
    TYPE = 1
    RULING = 2
    BLAZE = 3
    GRATING = 4
    SPEED = 5
    SIZE = 6
    GRATING_COUNT = 13
    UNITS = 14
    SERIAL_NO = 19


class Unit(Enum):
    MICRONS = 0
    NANOMETERS = 1
    ANGSTROMS = 2


# TODO: see if there's a way to make this usage easier
ValidSpeeds: Mapping = MappingProxyType(
    {
        3600: (333, 166, 83, 41, 20, 10, 5, 2, 1, 0),
        2400: (500, 250, 125, 62, 31, 15, 7, 3, 1, 0),
        1800: (666, 332, 166, 82, 40, 20, 10, 4, 2, 0),
        1200: (1000, 500, 250, 125, 62, 31, 15, 7, 3, 1),
        600: (2000, 1000, 500, 250, 124, 62, 30, 14, 6, 2),
        300: (4000, 2000, 1000, 496, 248, 120, 56, 24, 8),
        150: (8000, 4000, 2000, 1000, 496, 248, 120, 56, 24, 8),
        74: (16000, 8000, 4000, 2000, 992, 496, 240, 112, 48, 16),
    }
)


class StatusMessage:
    def __init__(self, status_byte: bytes, scan_mode=False):
        self._status_byte = status_byte
        self._scan_mode = scan_mode

    @property
    def accepted(self):
        return not bool(self._status_byte >> 7 & 1)

    @property
    def action(self):
        return not bool(self._status_byte >> 6 & 1)

    @property
    def specifier(self):
        if self.accepted:
            return "correct"

        match self._status_byte >> 5 & 1:
            case 0:
                return "too large"
            case 1:
                return "too small"

    @property
    def scan_sign(self):
        match self._status_byte >> 4 & 1:
            case 0:
                return "positive"
            case 1:
                return "negative"

    @property
    def order_sign(self):
        match self._status_byte >> 3 & 1:
            case 0:
                return "positive"
            case 1:
                return "negative"

    @property
    def units(self):
        return Unit(self._status_byte & 0b111)
        # match self._status_byte & 0b111:
        #     case 0:
        #         return Units.ANGSTROMS
        #     case 1:
        #         return Units.NANOMETERS
        #     case 2:
        #         return Units.MICRONS
        #     case _:
        #         raise ValueError("Unknown unit type")

    @property
    def scan_start_accepted(self):
        if not self._scan_mode:
            return None
        match self._status_byte >> 6 & 1:
            case 0:
                return True
            case 1:
                return False

    @property
    def scan_end_accepted(self):
        if not self._scan_mode:
            return None
        match self._status_byte >> 5 & 1:
            case 0:
                return True
            case 1:
                return False

    def __repr__(self):
        message = "CM110 Status: \n"
        message += f"Message Accepted: {self.accepted} \n"
        if self._scan_mode:
            message += f"Scan Start Accepted: {self.scan_start_accepted} \n"
            message += f"Scan End Accepted: {self.scan_end_accepted} \n"
        else:
            message += f"Action Required: {self.action} \n"
            message += f"Specifier Size: {self.specifier} \n"
        message += f"Scan Sign: {self.scan_sign} \n"
        message += f"Order Sign: {self.order_sign} \n"
        message += f"Units: {self.units.name} \n"
        return message


class CM110:
    CM110_SERIAL_KWARGS: Mapping = MappingProxyType(
        {
            "baudrate": 9600,
            "parity": serial.PARITY_NONE,
            "stopbits": serial.STOPBITS_ONE,
            "bytesize": serial.EIGHTBITS,
            "timeout": 0.5,
        }
    )

    @classmethod
    def discover(self, serial_number=None) -> "CM110":
        """Attempt automatic discovery of the CM110 serial port
        and return the CM110 object.

        Returns
        -------
        CM110
            A successfully automatic CM110 object.

        Raises
        ------
        serial.SerialException
            If no serial port can be automatically linked.
        """
        port_list = serial.tools.list_ports.comports()

        if len(port_list) == 0:
            raise serial.SerialException("No serial ports found on this machine")

        for p in port_list:
            try:
                sp = serial.Serial(p.device, **self.CM110_SERIAL_KWARGS)
                sp.read_all()
                monochrometer = CM110(sp)
                serial_no, status = monochrometer.query(Query.SERIAL_NO)

                if status.accepted:
                    if not serial_number:
                        return monochrometer

                    if serial_no == serial_number:
                        return monochrometer
                    else:
                        sp.close()
                        continue

            except Exception:
                sp.close()
                continue
        raise serial.SerialException(
            textwrap.dedent(
                """Could not connect to any CM110 monochrometer.
                Check connection and device power."""
            )
        )

    def __init__(self, port=None):
        if isinstance(port, str):
            self._port = serial.Serial(port_name, **self.CM110_SERIAL_KWARGS)

        self._port = port
        self._clear_buffer()

    def close(self):
        self._port.close()

    def _clear_buffer(self):
        self._port.read_all()

    def _block_on_echo(self, timeout_s=20):
        """
        Block code execution while CM110 is busy and can't reply.
        """
        start_time = time.time()
        current_time = start_time
        while current_time - start_time < timeout_s:
            if self.echo():
                return
            current_time = time.time()
            time.sleep(1)

        raise TimeoutError(f"No response from CM110 in {timeout_s} seconds")

    def send_command(self, command: int, arg_bytes: bytes = None):
        message = command.value.to_bytes(1, "big")
        if arg_bytes:
            message += arg_bytes
        self._port.write(message)
        response = self._port.read_until(LINE_END)
        return response.rstrip(LINE_END)  # remove line end

    def reset(self):
        self.send_command(Cmd.RESET, b"\xff\xff")  # 255, 255

    def echo(self):
        results = self.send_command(Cmd.ECHO)
        return results == Cmd.ECHO.value.to_bytes(1, "big")

    def size(self, size: int):
        # values are in the current unit.
        if size > 127:
            raise ValueError(f"Size is too large, expected 127, got {size}")
        if size < -127:
            raise ValueError(f"Size is too small, expected -127, got {size}")
        if size == 0:
            raise ValueError(f"Size is not allowed to be zero")

        # positive values are 0-127
        # negative values are 128-255
        if size < 0:
            size = -size + 128

        args = size.to_bytes(1, "big")
        results = self.send_command(Cmd.SIZE, args)
        return self.status(results)

    def step(self):
        results = self.send_command(Cmd.STEP)
        return self.status(results)

    def speed(self, speed: int):
        ruling, _ = self.query(Query.RULING)
        valid_speeds = ValidSpeeds[ruling]
        if speed not in valid_speeds:
            raise ValueError(f"Invalid speed, choices are: {valid_speeds}")

        args = speed.to_bytes(2, "big")
        results = self.send_command(Cmd.SPEED, args)
        return self.status(results)

    def status(self, results: bytes):
        # isolates the status byte of a full message
        try:
            status_byte = results[-1]
        except IndexError:
            return None
        return StatusMessage(status_byte)

    def units(self, unit: Unit):
        args = unit.value.to_bytes(1, "big")
        results = self.send_command(Cmd.UNITS, args)
        return self.status(results)

    def select(self, grating_no: int):
        if 3 > grating_no < 1:
            raise ValueError(f"Invalid grating number {grating_no}, choices are 1 or 2")

        args = grating_no.to_bytes(1, "big")
        results = self.send_command(Cmd.SELECT, args)

        self._block_on_echo()

        return self.status(results)

    def order(self, order: int):
        if order not in [-1, 1]:
            raise ValueError(f"Invalid order direction {order}, choices are -1 or 1")

        # device represents negative orders with 254
        if order == -1:
            order = 254

        args = order.to_bytes(1, "big")
        results = self.send_command(Cmd.ORDER, args)
        return self.status(results)

    def zero_inc(self):
        results = self.send_command(Cmd.INC)
        return self.status(results)

    def zero_dec(self):
        results = self.send_command(Cmd.DEC)
        return self.status(results)

    def zero_save(self):
        results = self.send_command(Cmd.ZERO, b"\x01")
        return self.status(results)

    def calibrate(self, wavelength: int):
        args = wavelength.to_bytes(2, "big")
        results = self.send_command(Cmd.CALIBRATE, args)
        return self.status(results)

    def goto(self, wavelength: int):
        args = wavelength.to_bytes(2, "big")
        results = self.send_command(Cmd.GOTO, args)
        return self.status(results)

    def scan(self, start: int, end: int):
        start_bytes = start.to_bytes(2, "big")
        end_bytes = end.to_bytes(2, "big")

        args = start_bytes + end_bytes
        results = self.send_command(Cmd.SCAN, args)

        # CTS line is used to indicate when scan starts and stops
        while not self._port.cts:
            time.sleep(1)

        return self.status(results)

    def query(self, query: Query):
        args = query.value.to_bytes(1, "big")
        results = self.send_command(Cmd.QUERY, args)
        value = int.from_bytes(results[0:2], "big")

        if query is Query.UNITS:
            value = Unit(value)

        return value, self.status(results)


# Example Usage:
if __name__ == "__main__":
    mc = CM110.discover()

    # Example commands
    status = mc.goto(400)
    print(status)

    value, status = mc.query(Query.POSITION)
    print(value)
    # print(status)
