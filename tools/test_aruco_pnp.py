import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

test_code = """
import cv2, numpy as np

# Camera matrix for OV5647 640x480
fx = 530.0
fy = 530.0
cx = 320.0
cy = 240.0
cam_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

marker_size_cm = 10.0 # 10 cm printed tag

# 3D points of square marker in marker coordinate frame
obj_pts = np.array([
    [-marker_size_cm / 2,  marker_size_cm / 2, 0],
    [ marker_size_cm / 2,  marker_size_cm / 2, 0],
    [ marker_size_cm / 2, -marker_size_cm / 2, 0],
    [-marker_size_cm / 2, -marker_size_cm / 2, 0]
], dtype=np.float32)

# Simulated corners for a marker 120 cm away, slightly to the right (x=400)
# W_px = 10 * 530 / 120 = 44.1 px
cx_img = 400.0
cy_img = 240.0
hw = 22.0
img_pts = np.array([
    [cx_img - hw, cy_img - hw],
    [cx_img + hw, cy_img - hw],
    [cx_img + hw, cy_img + hw],
    [cx_img - hw, cy_img + hw]
], dtype=np.float32)

ret, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam_matrix, dist_coeffs)
dist_cm = np.linalg.norm(tvec)
bearing_deg = np.degrees(np.arctan2(tvec[0][0], tvec[2][0]))
print(f'PnP Result: Distance = {dist_cm:.1f} cm, Bearing = {bearing_deg:+.1f} deg, (X={tvec[0][0]:.1f}, Z={tvec[2][0]:.1f})')
"""

cmd = f"/home/alfin/autonomous_car/venv/bin/python3 -c \"{test_code}\""
stdin, stdout, stderr = client.exec_command(cmd)
print("OUT:\n" + stdout.read().decode())
print("ERR:\n" + stderr.read().decode())
client.close()
