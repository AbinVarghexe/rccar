import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

test_code = """
import cv2
print('cv2 version:', cv2.__version__)
print('has aruco:', hasattr(cv2, 'aruco'))
if hasattr(cv2, 'aruco'):
    # Test dictionary creation
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        print('DICT_4X4_50 ready:', dictionary is not None)
    except Exception as e:
        print('dict error:', e)
"""

cmd = f"/home/alfin/autonomous_car/venv/bin/python3 -c \"{test_code}\""
stdin, stdout, stderr = client.exec_command(cmd)
print("OUT:\n" + stdout.read().decode())
print("ERR:\n" + stderr.read().decode())
client.close()
