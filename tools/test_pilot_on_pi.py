import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

test_script = """
import sys
sys.path.insert(0, '/home/alfin/autonomous_car')
import numpy as np
import cv2
import time
from behavioral_cloning import PilotModel

print('Initializing PilotModel...')
t0 = time.time()
pilot = PilotModel('models/SidewalkPilot-v3.1b.onnx')
ok = pilot.load()
print(f'Pilot loaded: {ok}, backend: {pilot.backend}, error: {pilot.error}')

dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
t1 = time.time()
steer, throttle = pilot.predict(dummy_frame)
t2 = time.time()
print(f'Prediction: steer={steer:.3f}, throttle={throttle:.3f}, inference_time={(t2-t1)*1000:.1f}ms')
"""

cmd = f"/home/alfin/autonomous_car/venv/bin/python3 -c \"{test_script}\""
stdin, stdout, stderr = client.exec_command(cmd)
print("OUT:", stdout.read().decode())
print("ERR:", stderr.read().decode())
client.close()
