import serial

ser = serial.Serial("/dev/tty.usbmodem101", 115200)

with open("serial_output.txt", "w") as file:
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(data)
            file.write(data + '\n')
            file.flush()