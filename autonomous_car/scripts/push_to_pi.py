#!/usr/bin/env python3
"""
push_to_pi.py - Direct SSH/SFTP Deployer & Service Manager for Raspberry Pi.

Features:
- Directly pushes local updates to Raspberry Pi via SSH/SFTP (bypassing GitHub fetch/pull).
- Tracks every file posted in .pi_deploy_manifest.json.
- Records timestamp, git commit, branch, and working tree git status in pi_deployment.log.
- Optionally restarts rccar.service on the Pi and checks service health.
"""

import os
import sys
import time
import json
import hashlib
import argparse
import subprocess
from datetime import datetime

try:
    import paramiko
except ImportError:
    print("[ERROR] paramiko library is required. Install with: pip install paramiko")
    sys.exit(1)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

# Default Raspberry Pi Connection Credentials
DEFAULT_HOST = os.environ.get("PI_HOST", "10.91.16.78")
DEFAULT_PORT = 22
DEFAULT_USER = "alfin"
DEFAULT_PASS = "123456"
REMOTE_BASE_DIR = "/home/alfin/autonomous_car"

WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LOCAL_APP_DIR = os.path.join(WORKSPACE_ROOT, "autonomous_car")
LOG_FILE = os.path.join(WORKSPACE_ROOT, "pi_deployment.log")
MANIFEST_FILE = os.path.join(WORKSPACE_ROOT, ".pi_deploy_manifest.json")

# Directories/files to exclude from syncing
EXCLUDE_EXTENSIONS = {".pyc", ".pyo", ".pyd", ".jpg", ".jpeg", ".png", ".mjpeg", ".mp4", ".h264", ".pt"}
EXCLUDE_DIRS = {"__pycache__", "venv", ".venv", "media", ".git", ".idea", ".vscode"}


def get_file_md5(filepath: str) -> str:
    hasher = hashlib.md5()
    try:
        with open(filepath, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                hasher.update(chunk)
        return hasher.hexdigest()
    except Exception:
        return ""


def get_git_info() -> dict:
    info = {
        "branch": "unknown",
        "commit": "unknown",
        "commit_msg": "",
        "status_summary": "",
        "is_dirty": False
    }
    try:
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=WORKSPACE_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
        info["branch"] = branch

        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=WORKSPACE_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
        info["commit"] = commit

        msg = subprocess.check_output(
            ["git", "log", "-1", "--pretty=%B"],
            cwd=WORKSPACE_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
        info["commit_msg"] = msg

        status = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=WORKSPACE_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
        info["status_summary"] = status
        info["is_dirty"] = len(status) > 0
    except Exception as e:
        info["git_error"] = str(e)
    return info


def load_manifest() -> dict:
    if os.path.exists(MANIFEST_FILE):
        try:
            with open(MANIFEST_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def save_manifest(manifest: dict) -> None:
    try:
        with open(MANIFEST_FILE, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)
    except Exception as e:
        print(f"[WARN] Failed to write manifest: {e}")


def log_event(message: str) -> None:
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    formatted = f"[{timestamp}] {message}"
    print(formatted)
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(formatted + "\n")
    except Exception:
        pass


def collect_target_files(specific_files=None, diff_only=False, previous_manifest=None):
    files_to_push = []

    if specific_files:
        for f in specific_files:
            abs_p = os.path.abspath(f) if os.path.isabs(f) else os.path.abspath(os.path.join(WORKSPACE_ROOT, f))
            if os.path.exists(abs_p) and os.path.isfile(abs_p):
                rel_p = os.path.relpath(abs_p, LOCAL_APP_DIR)
                files_to_push.append((abs_p, rel_p))
            else:
                print(f"[WARN] File not found: {f}")
        return files_to_push

    # Walk autonomous_car tree
    for root, dirs, files in os.walk(LOCAL_APP_DIR):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        for file in files:
            ext = os.path.splitext(file)[1].lower()
            if ext in EXCLUDE_EXTENSIONS:
                continue

            abs_path = os.path.join(root, file)
            rel_path = os.path.relpath(abs_path, LOCAL_APP_DIR)

            if diff_only and previous_manifest:
                curr_md5 = get_file_md5(abs_path)
                prev_record = previous_manifest.get("files", {}).get(rel_path.replace("\\", "/"))
                if prev_record and prev_record.get("md5") == curr_md5:
                    continue  # Unchanged

            files_to_push.append((abs_path, rel_path))

    return files_to_push


def remote_mkdir_p(sftp, remote_directory):
    dirs_to_create = []
    current = remote_directory
    while current not in ("", "/"):
        try:
            sftp.stat(current)
            break
        except FileNotFoundError:
            dirs_to_create.append(current)
            current = os.path.dirname(current)

    while dirs_to_create:
        d = dirs_to_create.pop()
        try:
            sftp.mkdir(d)
        except Exception:
            pass


def execute_ssh_command(client, command, sudo_pass=None):
    if sudo_pass and "sudo" in command:
        command = f"echo '{sudo_pass}' | sudo -S {command}"
    stdin, stdout, stderr = client.exec_command(command)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out, err


def push_to_pi(host=DEFAULT_HOST, port=DEFAULT_PORT, user=DEFAULT_USER, password=DEFAULT_PASS,
               files=None, diff_only=False, restart_service=False):
    log_event("=" * 60)
    log_event(f"Starting Direct Board Push to Raspberry Pi: {user}@{host}:{port}")

    git_info = get_git_info()
    log_event(f"Git Branch: {git_info['branch']} | Commit: {git_info['commit'][:8]} | Dirty: {git_info['is_dirty']}")
    if git_info["status_summary"]:
        log_event(f"Git Status Changes:\n{git_info['status_summary']}")

    manifest = load_manifest()
    files_to_push = collect_target_files(files, diff_only=diff_only, previous_manifest=manifest)

    if not files_to_push:
        log_event("No modified files to push. Pi is already up to date with local changes.")
        if restart_service:
            log_event("[*] Restarting rccar.service on Raspberry Pi...")
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            connected = False
            for attempt in range(3):
                try:
                    client.connect(host, port=port, username=user, password=password, timeout=15)
                    connected = True
                    break
                except Exception as e:
                    if attempt == 2:
                        log_event(f"[ERROR] Service restart failed to connect: {e}")
                    time.sleep(2)
            if connected:
                try:
                    execute_ssh_command(client, "sudo systemctl restart rccar.service", sudo_pass=password)
                    time.sleep(1.5)
                    status_out, _ = execute_ssh_command(client, "systemctl is-active rccar.service")
                    log_event(f"rccar.service status: {status_out.strip()}")
                except Exception as e:
                    log_event(f"[ERROR] Service restart command failed: {e}")
                finally:
                    client.close()
        return True

    log_event(f"Files to deploy ({len(files_to_push)} file(s)):")
    for _, rel in files_to_push:
        log_event(f"  -> autonomous_car/{rel.replace(chr(92), '/')}")

    # Connect via SSH with retry
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    connected = False
    last_err = None
    for attempt in range(3):
        try:
            client.connect(host, port=port, username=user, password=password, timeout=15)
            connected = True
            break
        except Exception as e:
            last_err = e
            time.sleep(2)

    if not connected:
        log_event(f"[ERROR] Failed to connect to Pi: {last_err}")
        return False

    sftp = client.open_sftp()
    uploaded_records = {}

    try:
        for abs_p, rel_p in files_to_push:
            rel_unix = rel_p.replace("\\", "/")
            remote_path = f"{REMOTE_BASE_DIR}/{rel_unix}"
            remote_dir = os.path.dirname(remote_path)

            remote_mkdir_p(sftp, remote_dir)
            sftp.put(abs_p, remote_path)
            md5_hash = get_file_md5(abs_p)
            file_size = os.path.getsize(abs_p)

            uploaded_records[rel_unix] = {
                "md5": md5_hash,
                "size_bytes": file_size,
                "deployed_at": datetime.now().isoformat(),
                "remote_path": remote_path
            }
            log_event(f"[DEPLOYED] {rel_unix} ({file_size} bytes)")

        # Update manifest
        if "files" not in manifest:
            manifest["files"] = {}
        manifest["files"].update(uploaded_records)
        manifest["last_deploy"] = {
            "timestamp": datetime.now().isoformat(),
            "target_host": host,
            "git_commit": git_info["commit"],
            "git_branch": git_info["branch"],
            "files_pushed_count": len(uploaded_records),
        }
        save_manifest(manifest)
        log_event(f"[SUCCESS] Tracked {len(uploaded_records)} files in manifest.")

    finally:
        sftp.close()

    if restart_service:
        log_event("[*] Restarting rccar.service on Raspberry Pi...")
        out, err = execute_ssh_command(client, "sudo systemctl restart rccar.service", sudo_pass=password)
        time.sleep(1.5)
        status_out, _ = execute_ssh_command(client, "systemctl is-active rccar.service")
        status_cleaned = status_out.strip()
        log_event(f"rccar.service status: {status_cleaned}")

    client.close()
    log_event("Push completed successfully.")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Push local code directly to Raspberry Pi over SSH/SFTP.")
    parser.add_argument("--ip", default=DEFAULT_HOST, help="Raspberry Pi IP address")
    parser.add_argument("--user", default=DEFAULT_USER, help="SSH username")
    parser.add_argument("--password", default=DEFAULT_PASS, help="SSH password")
    parser.add_argument("--files", nargs="*", default=None, help="Specific file(s) to push")
    parser.add_argument("--diff", action="store_true", help="Push only files modified since last deployment")
    parser.add_argument("--restart", action="store_true", help="Restart rccar.service on Pi after pushing")
    parser.add_argument("--status", action="store_true", help="Check rccar.service status on Pi")

    args = parser.parse_args()

    if args.status:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(args.ip, port=DEFAULT_PORT, username=args.user, password=args.password, timeout=5)
        out, err = execute_ssh_command(client, "systemctl status rccar.service --no-pager")
        print(out)
        client.close()
        sys.exit(0)

    success = push_to_pi(
        host=args.ip,
        port=DEFAULT_PORT,
        user=args.user,
        password=args.password,
        files=args.files,
        diff_only=args.diff,
        restart_service=args.restart
    )
    sys.exit(0 if success else 1)
