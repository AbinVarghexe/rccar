"""
tools/run_local_cockpit.py — Runs the Mission Cockpit locally on Windows using the Inbuilt Webcam.
Access URL: http://127.0.0.1:5555
"""

import os
import sys

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

import autonomous_car.scripts.test_dashboard as dash
import webbrowser

def main():
    print("=" * 70)
    print("STARTING LOCAL AUTONOMOUS COCKPIT (INBUILT WEBCAM)")
    print("=" * 70)
    print("URL: http://127.0.0.1:5555")
    print("Camera: Inbuilt Webcam (cv2.VideoCapture(0))")
    print("Show ArUco tags or phone screen with tags to test live 3D tracking.")
    print("Press Ctrl+C to terminate.")
    print("=" * 70)
    
    dash.connect_serial()
    dash.threading.Thread(target=dash.reader_thread, daemon=True).start()
    dash.threading.Thread(target=dash.camera_vision_thread, daemon=True).start()
    
    server = dash.ThreadedHTTPServer(("127.0.0.1", 5555), dash.DashboardHandler)
    
    # Open browser automatically after 1 second
    def open_browser():
        dash.time.sleep(1.2)
        try:
            webbrowser.open("http://127.0.0.1:5555")
        except Exception:
            pass
    dash.threading.Thread(target=open_browser, daemon=True).start()
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping local cockpit...")
        server.server_close()

if __name__ == "__main__":
    main()
