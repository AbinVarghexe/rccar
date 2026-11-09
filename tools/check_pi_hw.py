import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('10.91.16.78', port=22, username='alfin', password='123456', timeout=10)

for cmd in ['lsusb', 'ls -la /dev/serial* /dev/ttyACM* /dev/ttyUSB* 2>&1']:
    stdin, stdout, stderr = client.exec_command(cmd)
    print(f"=== {cmd} ===")
    print(stdout.read().decode())

client.close()
