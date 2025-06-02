For the mqtt communication install the dependencies:

	sudo apt update
	sudo apt install \
	  build-essential \
	  cmake \
	  mosquitto\
	  mosquitto-clients\
	  libpaho-mqttpp3-dev \
	  libpaho-mqtt3as-dev \
	  libstdc++-fs \
  
 Alternatively compile them from source:

	git clone https://github.com/eclipse/paho.mqtt.c.git
	cd paho.mqtt.c
	mkdir build && cd build
	cmake -DPAHO_BUILD_STATIC=ON -DPAHO_ENABLE_TESTING=OFF ..
	make && sudo make install
	
	git clone https://github.com/eclipse/paho.mqtt.c.git
	cd paho.mqtt.c
	mkdir build && cd build
	cmake -DPAHO_BUILD_STATIC=ON -DPAHO_ENABLE_TESTING=OFF ..
	make && sudo make install

Launch the communication broker:
	moquitto -v (verbose option)

For the ctrl_station package run:

ros2 run ctrl_station mqtt_client
ros2 run ctrl_station terminal
ros2 run ctrl_station cmd_dispatcher

Instead, from the json_utils repo (on branch marco_branch)
	
	mkdir build && cd build
	cmake ..
	sudo make install

Then run ./mqtt_server

HOW TO USE IT:

- On the mqtt_server terminal follow the menu and insert desired latitude and longitude,
  then these value are sended automatically.

- After that, from the main terminal select the number of the tbm you want to execute.

- At the end, still from the main terminal, you can send the mission command to the AUV.

- You can repeat these step among various tests.

If you want to change info about the run, like buoys area or pipelines you've change the conf file
in /ctrl_station/conf (is the same in the mission_ctrl node but i was having issues reading the one there).

