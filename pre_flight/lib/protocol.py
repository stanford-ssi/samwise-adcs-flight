"""USB CDC protocol client for ``samwise-adcs-preflight.uf2``."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Iterable

import serial
from serial.tools import list_ports


@dataclass(frozen=True)
class Packet:
    kind: str
    fields: tuple[str, ...]
    raw_line: str

    @property
    def timestamp_ms(self) -> int | None:
        if self.kind not in {"MAG", "IMU", "SUN"} or not self.fields:
            return None
        return int(self.fields[0])


def parse_packet(line: str) -> Packet | None:
    """Parse one pre-flight CSV line; ignore unrelated driver chatter."""
    text = line.strip()
    if not text.startswith("PF,"):
        return None
    parts = text.split(",")
    if len(parts) < 2:
        return None
    return Packet(parts[1], tuple(parts[2:]), text)


def packet_vector(packet: Packet, section: str = "raw") -> tuple[float, float, float]:
    if packet.kind == "MAG":
        offset = 1 if section == "raw" else 4
    elif packet.kind == "IMU":
        offset = 1 if section == "gyro" else 4
    else:
        raise ValueError(f"{packet.kind} does not contain a 3-vector")
    values = tuple(float(value) for value in packet.fields[offset : offset + 3])
    if len(values) != 3:
        raise ValueError(f"Malformed {packet.kind} packet: {packet.raw_line}")
    return values


def packet_sun(packet: Packet) -> tuple[int, ...]:
    if packet.kind != "SUN" or len(packet.fields) != 17:
        raise ValueError(f"Malformed SUN packet: {packet.raw_line}")
    return tuple(int(value) for value in packet.fields[1:])


def discover_ports() -> list[tuple[str, str]]:
    """Return likely serial ports, preferring USB CDC devices."""
    ports = []
    for item in list_ports.comports():
        label = item.description or "serial device"
        ports.append((item.device, label))
    return sorted(
        ports,
        key=lambda item: (
            0 if any(token in item[0].lower() for token in ("usbmodem", "ttyacm", "com")) else 1,
            item[0],
        ),
    )


def resolve_port(requested: str) -> str:
    if requested.lower() != "auto":
        return requested
    ports = discover_ports()
    if not ports:
        raise RuntimeError("No serial ports found. Connect the ADCS board over USB and retry.")
    likely = [
        port
        for port, description in ports
        if any(
            token in f"{port} {description}".lower()
            for token in ("usbmodem", "ttyacm", "pico", "rp2350", "usb serial")
        )
    ]
    if len(likely) == 1:
        return likely[0]
    if len(ports) == 1:
        return ports[0][0]
    rendered = "\n".join(f"  {port}: {description}" for port, description in ports)
    raise RuntimeError(
        "More than one serial port is available; pass --port explicitly:\n" + rendered
    )


class PreflightLink:
    def __init__(self, port: str = "auto", baud: int = 115200, timeout: float = 0.2):
        self.port = resolve_port(port)
        self.serial = serial.Serial(self.port, baudrate=baud, timeout=timeout)
        self.serial.reset_input_buffer()
        time.sleep(0.25)

    def close(self) -> None:
        if self.serial.is_open:
            try:
                try:
                    self.stop()
                except serial.SerialException:
                    # A board reset or unplug closes the transport before a
                    # final STOP can be delivered. The device-side arm timer
                    # still fails closed after 30 seconds.
                    pass
            finally:
                self.serial.close()

    def __enter__(self) -> "PreflightLink":
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()

    def send(self, command: str) -> None:
        self.serial.write((command.strip() + "\n").encode("ascii"))
        self.serial.flush()

    def stop(self) -> None:
        if self.serial.is_open:
            self.send("STOP")

    def read_packet(self, deadline: float | None = None) -> Packet | None:
        while deadline is None or time.monotonic() < deadline:
            line = self.serial.readline().decode("utf-8", errors="replace")
            if not line:
                return None
            packet = parse_packet(line)
            if packet is not None:
                return packet
        return None

    def wait_for(
        self,
        predicate: Callable[[Packet], bool],
        timeout: float = 5.0,
    ) -> Packet:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            packet = self.read_packet(deadline)
            if packet is not None and predicate(packet):
                return packet
        raise TimeoutError("Timed out waiting for the pre-flight board")

    def verify(self) -> int:
        self.send("PING")
        packet = self.wait_for(lambda item: item.kind == "PONG", timeout=4.0)
        if not packet.fields:
            raise RuntimeError("Board returned a malformed PONG")
        version = int(packet.fields[0])
        if version != 1:
            raise RuntimeError(f"Unsupported pre-flight protocol version {version}")
        return version

    def status(self) -> dict[str, bool]:
        self.send("STATUS")
        packet = self.wait_for(lambda item: item.kind == "STATUS")
        result: dict[str, bool] = {}
        for field in packet.fields:
            if "=" in field:
                key, value = field.split("=", 1)
                result[key] = value == "1"
        return result

    def imu_zero(self) -> tuple[float, float, float]:
        self.send("CONFIG")
        packet = self.wait_for(
            lambda item: item.kind == "CONFIG" and item.fields[:1] == ("imu_zero",)
        )
        values = tuple(float(value) for value in packet.fields[1:4])
        if len(values) != 3:
            raise RuntimeError("Board returned malformed IMU calibration metadata")
        return values  # type: ignore[return-value]

    def collect(
        self,
        kind: str,
        count: int,
        rate_hz: int,
        extractor: Callable[[Packet], Iterable[float]],
        timeout: float | None = None,
        progress: Callable[[int, int], None] | None = None,
    ) -> list[tuple[float, ...]]:
        kind = kind.upper()
        self.send(f"STREAM {kind} {rate_hz}")
        self.wait_for(lambda item: item.kind == "OK" and item.fields[:1] == ("STREAM",))
        deadline = time.monotonic() + (timeout or max(10.0, count / max(rate_hz, 1) * 3.0))
        samples: list[tuple[float, ...]] = []
        try:
            while len(samples) < count and time.monotonic() < deadline:
                packet = self.read_packet(deadline)
                if packet is None or packet.kind != kind:
                    continue
                values = tuple(float(value) for value in extractor(packet))
                if values and all(math.isfinite(value) for value in values):
                    samples.append(values)
                    if progress:
                        progress(len(samples), count)
        finally:
            self.send("STREAM OFF 1")
        if len(samples) < count:
            raise TimeoutError(f"Collected only {len(samples)}/{count} valid {kind} samples")
        return samples
