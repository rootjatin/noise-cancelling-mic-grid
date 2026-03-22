# ESP32-CAM Upload Notes

Before uploading the code, **short GND and GPIO0 (IO0)** to put the ESP32-CAM into flashing mode.

## Steps

1. Connect a jumper wire between **GND** and **GPIO0 (IO0)**.
2. In the Arduino IDE, select:
   - **Board:** `ESP32 Wrover Module`
   - **Partition Scheme:** `Huge APP`
3. Upload the code to the ESP32-CAM.
4. After the upload is complete, **remove the jumper wire between GND and GPIO0**.
5. Press the **RST** button or power cycle the board.
6. After the ESP32-CAM connects to Wi-Fi, open **`http://<ip>`** in your browser to view the logs/output.

> Replace `<ip>` with the IP address shown in the Serial Monitor.
