import os
import sys
import paramiko

host = os.environ.get('PI_HOST', '10.91.16.78')
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(host, port=22, username='alfin', password='123456', timeout=10)

remote_cmd = sys.argv[1] if len(sys.argv) > 1 else 'uptime'
stdin, stdout, stderr = client.exec_command(remote_cmd)
out = stdout.read().decode('utf-8', errors='replace')
err = stderr.read().decode('utf-8', errors='replace')
if sys.stdout.encoding.lower() != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

if out:
    sys.stdout.write(out)
if err:
    sys.stderr.write(err)
client.close()
