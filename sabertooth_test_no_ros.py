import serial, time

ser = serial.Serial('/dev/sabertooth', 9600, timeout=1)
time.sleep(2)  # give it a moment to initialize

ADDRESS = 128

def send(cmd, val):
    val = max(0, min(127, val))
    chk = (ADDRESS + cmd + val) & 0x7F
    packet = bytes([ADDRESS, cmd, val, chk])
    print(f'Sending: {list(packet)}')
    ser.write(packet)

# Drive M1 forward at ~half speed
send(0, 3)
time.sleep(2)
# Stop
send(0, 0)
ser.close()