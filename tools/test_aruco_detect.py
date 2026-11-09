import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

test_code = """
import cv2, numpy as np
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_img = cv2.aruco.generateImageMarker(dictionary, 0, 200, 1)

# Place on a white canvas with border
canvas = np.ones((300, 300), dtype=np.uint8) * 255
canvas[50:250, 50:250] = marker_img

det = cv2.aruco.ArucoDetector(dictionary)
corners, ids, rejected = det.detectMarkers(canvas)
print('Detected IDs:', ids.flatten() if ids is not None else None)
print('Corners shape:', len(corners))
"""

cmd = f"/home/alfin/autonomous_car/venv/bin/python3 -c \"{test_code}\""
stdin, stdout, stderr = client.exec_command(cmd)
print("OUT:\n" + stdout.read().decode())
print("ERR:\n" + stderr.read().decode())
client.close()
