# PD Stepper MQTT Control

MQTT-based firmware for the PD Stepper motor controller.

## Features

- WiFi and MQTT configuration via web interface
- Velocity mode for continuous rotation
- Position mode for precise distance-based movement
- Encoder-based position tracking
- Home position setting
- Configurable motor parameters via MQTT

## Setup

### First Time Configuration

1. Upload the firmware to your PD Stepper board
2. On first boot, the device will create a WiFi access point named **"PD Stepper Setup"**
3. Connect to this network (no password required)
4. Open a web browser and navigate to **`192.168.4.1`**
5. Configure:
   - WiFi SSID and password
   - MQTT server address and port
   - MQTT username/password (optional)
   - MQTT base topic (default: "motor")
   - Motor settings (voltage, microsteps, current, etc.)
6. Click "Save Configuration"
7. The device will restart and connect to your WiFi network and MQTT broker

### Finding the IP Address

**Setup Mode (First Time or No WiFi Config):**
- Connect to WiFi network: **"PD Stepper Setup"** (no password)
- Open browser to: **`192.168.4.1`**

**After WiFi Configuration:**
Once the device is connected to your WiFi network, the web interface is still accessible at the device's WiFi IP address. Find the IP using one of these methods:

1. **Check Serial Monitor** (Easiest) - Connect via USB and open Serial Monitor (115200 baud). The device prints its IP address when it connects to WiFi. Look for: `"WiFi connected!"` followed by `"IP address: xxx.xxx.xxx.xxx"`
2. **Check your router's DHCP client list** - Log into your router's admin page and look for a device named "PD_Stepper_" followed by the MAC address
3. **Use an MQTT client** - If MQTT is working, check your MQTT broker's client list
4. **Network scanner** - Use a network scanning tool (like `nmap` or `arp -a` on command line) to find devices on your network
5. **Force back to setup mode** - If you can't find it, you can reset the WiFi configuration by clearing the flash memory or uploading a blank sketch, then re-uploading this firmware

**Note:** The web configuration interface is available in both setup mode (Access Point) and normal mode (WiFi connected), so you can reconfigure the device at any time by accessing its IP address in a web browser.

### Arduino IDE Settings

Before uploading, configure the Arduino IDE:

1. **Install ESP32 Board Support**: If you haven't already, install the ESP32 add-on in Arduino IDE. See [this tutorial](https://randomnerdtutorials.com/installing-esp32-arduino-ide-2-0/) for instructions.

2. **Board Selection**:
   - **Board**: `ESP32S3 Dev Module`
   - **USB CDC on Boot**: `Enabled`
   - **Upload Speed**: `921600` (or lower if you have issues)
   - **Port**: Select the COM port where your PD Stepper is connected

### Required Libraries

Install these libraries in the Arduino IDE:

- **WiFi** (included with ESP32)
- **ESPAsyncWebServer** - https://github.com/ESP32Async/ESPAsyncWebServer
- **AsyncTCP** - https://github.com/ESP32Async/AsyncTCP
- **PubSubClient** - https://github.com/knolleary/pubsubclient
- **TMC2209** - https://github.com/janelia-arduino/TMC2209
- **ArduinoJson** - https://github.com/bblanchon/ArduinoJson

## MQTT Topics

### Message Format Examples

| Topic | Message Type | Example Messages | Description |
|-------|--------------|------------------|-------------|
| `motor/mode` | String | `"velocity"`<br>`"move"`<br>`"position"` | Set operating mode: velocity for continuous rotation, move for one-time movement, position for continuous position maintenance |
| `motor/rpm` | Float/Integer | `"100"`<br>`"50.5"`<br>`"-30"` | Motor speed in RPM (only used in velocity mode) |
| `motor/direction` | String | `"cw"`<br>`"ccw"` | Motor direction: clockwise or counter-clockwise |
| `motor/home` | Boolean/String | `"true"`<br>`"1"`<br>`"false"` | Set current position as home (true/1 sets home, auto-resets to false) |
| `motor/conf` | JSON String | `{"steps_per_rotation":200,"distance_per_revolution":50.0}`<br>`{"steps_per_rotation":200,"distance_per_step":0.1,"acceleration":100.0}` | Motor configuration in JSON format |
| `motor/target` | String/Float/JSON | `"20"`<br>`"-10"`<br>`"20 15 mm/s"`<br>`"[100, 100]"`<br>`"[50, 25]"` | Target position in mm. JSON array format: `[target, velocity]` |
| `motor/enable` | Boolean/String | `"true"`<br>`"false"`<br>`"1"`<br>`"0"`<br>`"enabled"`<br>`"disabled"` | Enable/disable motor driver. When disabled, motor can be manually rotated and position is still readable |
| `motor/position` | Float (String) | `"25.50"`<br>`"-5.25"` | **Published topic** - Current position in mm from home (read-only) |

### Subscribed Topics (Control)

#### `motor/mode`
Set the motor operating mode.
- **Values**: 
  - `"velocity"` - Continuous rotation at set RPM
  - `"move"` - One-time movement to target position, stops when reached
  - `"position"` - Continuously maintains target position, actively corrects if moved manually
- **Type**: String

#### `motor/rpm`
Set the motor speed in RPM (only used when mode is "velocity").
- **Values**: Any positive or negative number
- **Type**: Float/Integer

#### `motor/direction`
Set the motor direction.
- **Values**: `"cw"` (clockwise) or `"ccw"` (counter-clockwise)
- **Type**: String

#### `motor/home`
Set the current position as home (zero position).
- **Values**: `true` or `1` to set home, automatically resets to `false`
- **Type**: Boolean/String

#### `motor/conf`
Motor configuration in JSON format.
- **Type**: JSON String
- **Example**:
  ```json
  {
    "steps_per_rotation": 200,
    "distance_per_revolution": 50.0,
    "acceleration": 100.0
  }
  ```
- **Fields**:
  - `steps_per_rotation`: Number of full steps per motor revolution (typically 200 or 400)
  - `distance_per_revolution`: Distance in mm that the motor travels in one full revolution
  - `acceleration`: Acceleration in mm/s² (for future use)
  - `distance_per_step`: Alternative to distance_per_revolution, directly specifies distance per step

#### `motor/target`
Set target position for move or position mode.
- **Type**: String/Float or JSON Array
- **Format**: 
  - Simple: `"20"` (target 20mm from home)
  - With velocity (string): `"20 15 mm/s"` (target 20mm at 15mm/s velocity)
  - JSON array: `"[100, 100]"` (target 100mm at 100mm/s velocity)
  - JSON array (target only): `"[50]"` (target 50mm at default 10mm/s velocity)
  - Negative: `"-10"` or `"[-10, 20]"` (target -10mm from home)
- **Note**: 
  - JSON array format: `[target, velocity]` where both are numbers
  - String format: Extracts the first number as the target distance. If "mm/s" is found, extracts velocity from the remaining text.
  - If velocity is not specified, defaults to 10 mm/s
  - In **move mode**: Motor moves to target once and stops
  - In **position mode**: Motor continuously maintains target position, correcting if moved manually

#### `motor/enable`
Enable or disable the motor driver.
- **Type**: Boolean/String
- **Values**: 
  - `"true"`, `"1"`, or `"enabled"` to enable the motor
  - `"false"`, `"0"`, or `"disabled"` to disable the motor
- **Use Case**: When disabled, the motor can be manually rotated (like a knob) and the encoder position will still be readable via `motor/position`. Useful for manual positioning, calibration, or taking measurements.

### Published Topics (Status)

#### `motor/position`
Current position in millimeters from home.
- **Type**: Float (published as string)
- **Update Rate**: Every 500ms

## Operating Modes

The firmware supports three operating modes:

1. **Velocity Mode** (`"velocity"`): Continuous rotation at a set RPM. Motor spins continuously until stopped or mode is changed.

2. **Move Mode** (`"move"`): One-time movement to a target position. Motor moves to the target and stops when reached. Useful for point-to-point movements.

3. **Position Mode** (`"position"`): Continuously maintains target position. Motor actively corrects if moved manually and continuously adjusts to maintain the target. Useful for position holding applications.

## Usage Examples

### Velocity Mode (Continuous Rotation)

```bash
# Set mode to velocity
mosquitto_pub -h your_mqtt_broker -t motor/mode -m "velocity"

# Set speed to 100 RPM clockwise
mosquitto_pub -h your_mqtt_broker -t motor/direction -m "cw"
mosquitto_pub -h your_mqtt_broker -t motor/rpm -m "100"

# Reverse direction
mosquitto_pub -h your_mqtt_broker -t motor/direction -m "ccw"
mosquitto_pub -h your_mqtt_broker -t motor/rpm -m "50"
```

### Move Mode (One-Time Movement)

```bash
# First, configure the motor (one time setup)
mosquitto_pub -h your_mqtt_broker -t motor/conf -m '{"steps_per_rotation":200,"distance_per_revolution":200.0}'

# Set home position
mosquitto_pub -h your_mqtt_broker -t motor/home -m "true"

# Set mode to move (one-time movement)
mosquitto_pub -h your_mqtt_broker -t motor/mode -m "move"

# Move to 20mm from home (stops when reached)
mosquitto_pub -h your_mqtt_broker -t motor/target -m "20"

# Move to -10mm from home at 15mm/s (stops when reached)
mosquitto_pub -h your_mqtt_broker -t motor/target -m "-10 15 mm/s"

# Using JSON array format
mosquitto_pub -h your_mqtt_broker -t motor/target -m "[100, 50]"
```

### Position Mode (Continuous Position Maintenance)

```bash
# Set mode to position (continuously maintains target)
mosquitto_pub -h your_mqtt_broker -t motor/mode -m "position"

# Set target to 50mm (motor will continuously maintain this position)
mosquitto_pub -h your_mqtt_broker -t motor/target -m "50"

# If you manually move the motor, it will automatically correct back to target
# Change target to 75mm (motor will move to new target and maintain it)
mosquitto_pub -h your_mqtt_broker -t motor/target -m "[75, 20]"
```

### Monitor Position

```bash
# Subscribe to position updates
mosquitto_sub -h your_mqtt_broker -t motor/position
```

### Manual Rotation Mode

```bash
# Disable motor to allow manual rotation
mosquitto_pub -h your_mqtt_broker -t motor/enable -m "false"

# Manually rotate the motor (position is still tracked via encoder)
# Monitor position in real-time:
mosquitto_sub -h your_mqtt_broker -t motor/position

# Re-enable motor when done
mosquitto_pub -h your_mqtt_broker -t motor/enable -m "true"
```

## Configuration Notes

- **Distance per revolution**: This is the linear distance (in mm) that your mechanism travels when the motor completes one full rotation. For example, if you have a lead screw with 2mm pitch, this would be 2.0.

- **Steps per rotation**: Typically 200 for 1.8° stepper motors or 400 for 0.9° motors.

- **Position tracking**: The firmware uses the AS5600 encoder for position feedback when available. If the encoder is not working, it falls back to step counting.

- **Home position**: Setting home (`motor/home = true`) sets the current encoder position as zero. All position commands are relative to this home position.

## Troubleshooting

### Can't access web interface
- **If in setup mode**: Connect to "PD Stepper Setup" WiFi network and go to `192.168.4.1`
- **If connected to WiFi**: 
  - Check Serial Monitor for the IP address (printed when WiFi connects)
  - Ensure you're on the same network as the device
  - Try accessing via the IP address shown in Serial Monitor
  - Check firewall settings that might block port 80
  - The web server runs on both AP mode and WiFi mode, so it should always be accessible

### Device doesn't connect to WiFi
- Check that WiFi credentials are correct
- Ensure the WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- The device will return to setup mode if it can't connect

### MQTT connection fails
- Verify MQTT broker address and port
- Check firewall settings
- Ensure MQTT broker is running and accessible
- Check username/password if authentication is required

### Position not accurate
- Ensure `motor/conf` is set correctly with accurate `distance_per_revolution`
- Verify encoder is working (check encoder position updates)
- Set home position when the mechanism is at a known location

### Motor doesn't move
- Check Power Good (PG) signal - motor won't enable if power is not good
- Verify motor is enabled in settings
- Check MQTT messages are being received (check serial output)

## Serial Commands

The firmware supports serial commands for debugging and configuration. Connect to the serial port at **115200 baud** to use these commands:

| Command | Description |
|---------|-------------|
| `reset` | Reset WiFi and MQTT settings. Device will restart in setup mode (Access Point) |
| `help` or `?` | Display available serial commands |

### Example Usage

1. Open Serial Monitor in Arduino IDE (115200 baud)
2. Type `reset` and press Enter
3. Device will clear WiFi/MQTT settings and restart in setup mode
4. Connect to "PD Stepper Setup" WiFi network to reconfigure

## Serial Debug Output

Connect to the serial port at 115200 baud to see:
- WiFi connection status
- MQTT connection status
- Received MQTT messages
- Configuration updates
- Position updates
- Serial command responses

