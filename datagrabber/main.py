import socket
import struct
import time

from serial import Serial


while True:
    ser = Serial(
        "/dev/rfcomm0",
        # "/dev/ttyACM0",
        baudrate=115200,
    )
    BLUETOOTH_NUM_MESSAGES_SIZE = 2
    BLUETOOOTH_ENTRY_SIZE = 14
    ANGLES_PER_FRAME = 6
    BLUETOOTH_ANGLE_FORMAT = "H"
    BLUETOOTH_ENTRY_FORMAT = "H" + ANGLES_PER_FRAME * BLUETOOTH_ANGLE_FORMAT
    UDP_IP = "192.168.178.1"
    UDP_PORT = 5555
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    rates = []
    start = None
    last = -1
    while True:
        header = ser.read_until(bytes([0xAA, 0xBB, 0xCC, 0xDD]))[:-4]
        sizebuf = ser.read(2)
        num_frames = int.from_bytes(sizebuf, byteorder="little")
        frame_size = BLUETOOOTH_ENTRY_SIZE * num_frames
        print("Waiting for ", num_frames, "frames with byte size", frame_size)
        # print(struct.calcsize(BLUETOOTH_ENTRY_FORMAT))
        buf = ser.read(frame_size)
        end = time.time()
        if start is not None:
            elapsed = end - start
            rate = num_frames * 6.0 / elapsed
            rates.append(rate)
            print(rate, sum(rates) / len(rates))
        start = end
        # total_packet = header + sizebuf + buf
        # print(total_packet)
        all_angles = set()
        for i in range(num_frames):
            entry_buf = buf[BLUETOOOTH_ENTRY_SIZE * i :][:BLUETOOOTH_ENTRY_SIZE]
            entry = struct.unpack(BLUETOOTH_ENTRY_FORMAT, entry_buf)
            angle_base = entry[0]
            angles = entry[1:]
            for j in range(ANGLES_PER_FRAME):
                angle = angle_base + j
                distance = angles[j]
                packet = struct.pack("H", angle) + struct.pack(
                    "ffff", float(distance), 0.0, 0.0, 0.0
                )
                sock.sendto(packet, (UDP_IP, UDP_PORT))
        # print("Num angles: ", len(all_angles))
        # print(rpm, angle_base, angles)
