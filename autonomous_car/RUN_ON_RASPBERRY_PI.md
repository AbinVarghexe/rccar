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
