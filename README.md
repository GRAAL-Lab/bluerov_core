# MQTT Communication Setup and Usage Guide

## 1. Install Dependencies

To enable MQTT communication, install the necessary system dependencies using `apt`:

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install \
  build-essential \
  cmake \
  mosquitto \
  mosquitto-clients \
  libpaho-mqttpp-dev \
  libpaho-mqtt-dev \
  nlohmann-json3-dev

```

### Alternative: Build MQTT Libraries from Source

If you prefer or need to compile the MQTT libraries manually:

#### Build C Library:

```bash
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
mkdir build && cd build
cmake -DPAHO_BUILD_STATIC=ON -DPAHO_ENABLE_TESTING=OFF ..
make
sudo make install
```

#### Build C++ Library:

```bash
git clone https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
mkdir build && cd build
cmake ..
make
sudo make install
```

## 2. Launch the MQTT Broker

Run Mosquitto in verbose mode:

```bash
sudo apt install mosquitto
mosquitto -v
```

## 3. Run ctrl\_station Package

Use the following commands to launch MQTT client utilities:

```bash
ros2 run ctrl_station mqtt_client
ros2 run ctrl_station terminal
ros2 run ctrl_station cmd_dispatcher
```

## 4. Build and Run mqtt\_server from `json_utils` Repo

```bash
git clone "https://bitbucket.org/isme_robotics/json_utils/src/marco_branch/"
git checkout marco_branch
mkdir build && cd build
cmake ..
sudo make install
```

Then launch the server:

```bash
./mqtt_server
```

## 5. How to Use the System

* In the `mqtt_server` terminal, follow the on-screen menu to input desired **latitude** and **longitude**. These values will be automatically sent to the system.
* In the main terminal, select the TBM you want to operate.
* After setup, issue the mission command to the Vectored BlueAUV from the main terminal.
* Repeat the above steps to run multiple tests as needed.

## 6. Modifying Mission Configuration

To change mission-related settings such as the **buoy area** or **pipeline configuration**, edit the config file:

```
/ctrl_station/conf/config.yaml
```

> **Note**: Although a similar config exists in the `mission_ctrl` node, it may not load correctly due to known issues.

---