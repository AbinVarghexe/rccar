# Automated 1-Command Raspberry Pi Setup & Execution Guide

---

## Automated 1-Command Installation

Copy the `autonomous_car` folder to your Raspberry Pi, open terminal inside the folder, and run:

```bash
bash setup.sh
```

The script automatically:
1. Updates package lists
2. Installs `Picamera2`, `OpenCV`, `PySerial` (`python3-serial`), `Flask`, `NumPy`, and `RPi.GPIO` via `apt` (takes <30 seconds)
3. Configures user permissions for Serial UART (`/dev/ttyACM0`) & GPIO pins
4. Creates a one-click `./run.sh` launcher script

---

## Starting the Software Stack

After running `setup.sh`, launch the car software by running:

```bash
./run.sh
```
or:
```bash
python3 main.py
```

---

## Dashboard Access

Open your web browser on your phone, laptop, or PC connected to the same Wi-Fi as the Raspberry Pi:

```text
http://<RASPBERRY_PI_IP>:5000
```
*(The IP address will be displayed in your terminal when `setup.sh` finishes)*

## Offline Region Navigation with ArUco Markers

The marker route controller is disabled by default. Print ArUco markers from
the `DICT_4X4_50` dictionary, place one at each region, and configure a route
through the dashboard API. For example, to drive from marker 10 to marker 20:

```bash
curl -X POST http://<RASPBERRY_PI_IP>:5000/api/marker/route \
  -H 'Content-Type: application/json' \
  -d '{"marker_ids":[10,20],"enabled":true}'
```

The car keeps the pan camera centered while driving, follows the active marker,
stops when that marker is reached, and advances to the next marker. If the
active marker is lost, the car holds `STOP`. HC-SR04 distance safety is checked
before marker steering and can still stop or reverse the car.

Check the route state with:

```bash
curl http://<RASPBERRY_PI_IP>:5000/api/marker/status
```

Clear the route and return to manual control with:

```bash
curl -X DELETE http://<RASPBERRY_PI_IP>:5000/api/marker/route
```
