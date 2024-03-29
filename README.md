# simple-slam

### Synopsis
Simple-slam is a cohesive system that performs 2D projection of a 3D mapping utilizing various different sensors. The project comprises of a vehiclar component, a client/server and navigation system guided with sensors. The vehicular component is reponsible for navigating around the room whereas the navigation system keeps track of your relative position based on your velocity, acceleration and rotation. These points are then streamed over to the HTTP server which then displays the position points on a graph. 


### Prerequistes
To ensure that you have the required technologies to run this project, please install the following:

- Golang - v1.22.1
- C++ - v17.0.0 compatible compiler

### Reproduction
To get started with the project, refer to the following steps:

1. Create a copy of `wifi_config.config` with your network details specific to your local network and rename the file to `wifi_config.h`.

Here is an example of `wifi_config.h` with sample network details. You can find your network's private `WEB_SERVER` (IPv4 address) by visiting your network settings and searching for its TCP/IP details.  
```
#define WIFI_SSID "iPhone"
#define WIFI_PASS "password"
#define WEB_SERVER "172.20.10.2"
```

2. Run the Golang server by changing directory into `server` and then running `go run cmd/main.go` in a new terminal.
3. Build and upload the platform.io project to your board.
4. After uploaded, you will need to begin the calibration process by pressing the button. This process begins with calibrating the mangetnometer, which can be done by rotating the board along all 3 axes until the green `LED1` goes off. Clicking once again will begin to calibrate the accelerometer/gyroscope which can be done by leaving the board untouched in the upright position.
5. After calibration is complete, `LED1` will blink 3 times to indicate that the system is ready to start collecting data. Click `BUTTON1` once more to begin the mapping process.