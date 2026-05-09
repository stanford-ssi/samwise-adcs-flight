import serial
import struct
import numpy as np
from cobs import cobs

def read_packet(ser):
    # Read until null terminator
    buf = bytearray()
    while True:
        byte = ser.read(1)
        if byte == b'\x00':
            break
        buf.extend(byte)
    return buf

def decode_packet(raw_cobs):
    try:
        decoded = cobs.decode(bytes(raw_cobs))
    except cobs.DecodeError:
        print(f"Decode error, discarding packet: {raw_cobs.hex()}")
        return None

    if len(decoded) < 7:  # minimum valid packet
        print(f"Packet too short: {len(decoded)} bytes")
        return None

    src, dst, seq, flags, type_, len_ = struct.unpack('6B', decoded[:6])

    if len(decoded) < 6 + len_ + 1:
        print(f"Payload length mismatch")
        return None

    payload = decoded[6:6 + len_]
    crc = decoded[6 + len_]
    return {'src': src, 'dst': dst, 'seq': seq,
            'flags': flags, 'type': type_, 'len': len_,
            'payload': payload, 'crc8': crc}

def parse_vec3_payload(payload):
    x, y, z = struct.unpack('<fff', payload)
    return (x, y, z)

if __name__ == "__main__":

    dev = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_3E9C33C72B0EF8EF-if00'
    vectors = []
    with serial.Serial(dev, timeout=1) as ser:
        while True:
            byte = ser.read(1)
            if byte == b'\x00':
                break
        try:
            while True:
                raw = read_packet(ser)
                if raw:
                    pkt = decode_packet(raw)
                    if pkt is None:
                        continue
                    vec = parse_vec3_payload(pkt['payload'])
                    vectors.append(vec)
        except KeyboardInterrupt:
            pass

    arr = np.array(vectors, dtype=np.float32)  # shape: (N, 3)
    np.save('vectors.npy', arr)
