import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

commands = [
    ("cv2 + dnn", "python3 -c 'import cv2; print(\"cv2:\", cv2.__version__, \"dnn:\", hasattr(cv2, \"dnn\"))'"),
    ("onnxruntime", "python3 -c 'import onnxruntime; print(\"onnx:\", onnxruntime.__version__)'"),
    ("tflite_runtime", "python3 -c 'import tflite_runtime; print(\"tflite:\", tflite_runtime.__version__)'"),
    ("venv python3", "/home/alfin/autonomous_car/venv/bin/python3 -c 'import onnxruntime; print(\"venv onnx:\", onnxruntime.__version__)'"),
    ("models list", "ls -lh /home/alfin/autonomous_car/models"),
]

for label, cmd in commands:
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    print(f"[{label}] -> {out if out else 'ERR: ' + err}")

client.close()
