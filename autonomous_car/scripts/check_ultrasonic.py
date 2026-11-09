import serial, time, os

for p in ['/dev/ttyACM1', '/dev/ttyACM0']:
    if os.path.exists(p):
        port = p
        break

print(f'Reading from {port} — put your hand in front of HC-SR04...')
s = serial.Serial(port, 115200, timeout=2)
time.sleep(2)
s.reset_input_buffer()

count = 0
while count < 15:
    l = s.readline().decode(errors='replace').strip()
    if l.startswith('DIST:'):
        dist = int(l.split(':')[1])
        bar = '█' * min(40, max(1, int((400 - dist) / 10)))
        print(f'  {l}  {bar}')
        count += 1
s.close()
