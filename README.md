# rr_quadx_ws

ROS2 workspace for Ryder Robot QuadX drone


## Overview 

![Overview](images/overview.png)

## Initial Setup

### Step 1 Preparing SD Card

![Imager](images/pi_writer.png)

Using Raspberry Pi Imager,  set the following:

| Option              | Value                             | Description                               |
|---------------------|-----------------------------------|-------------------------------------------|
| Raspberry Pi Device | RASPBERRY PI 5                    | Device that image will be installed on    |
| Operating System.   | UBUNTU SERVER 24.04.3 LTS (64bit) | Operating System on PI                    |
| Storage             | (See select)                      | Assigned reference of SD card on local OS |

### Step 2 Setup Network

After SD card is added to Raspberry Pi, set up a bridging network to perform next steps. Plug ethernet cable
into Pi and connect to local host computer. **Note that on Pi a reboot may be required before logging in works**.

Assuming that host computer is a mac go to 
* Apple Menu > System Settings > General > Sharing.
* Select Internet Sharing 
* Enable Internet Sharing by checking the box. Confirm when prompted.

Once logging into the Pi run the following commands:

```bash
# Get address of PI
ip a

# eth0 will be the address given by ip a command
sudo dhcpd eth0

# Test connection with a ping
ping 8.8.8.8
```
### Step 3 Update apt

On Pi

```bash
sudo apt update && sudo apt upgrade -y
```

Apply WiFi network setting

```bash
sudo netplan apply
```

### Step 4 Install Git

```bash
sudo apt install git -y
```

### Step 5 Create libcamera

```bash
sudo apt install -y build-essential  ninja-build meson software-properties-common cmake glib-2.0 doxygen graphviz python3-sphinx python3-sphinxcontrib.doxylink python3-pip
sudo apt install -y libcamera-dev libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev qtbase5-dev libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev libboost-program-options-dev libdrm-dev libexif-dev python3-ply qt6-base-dev libevent-dev v4l-utils
sudo apt install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-pulseaudio
mkdir -p system_ws && cd system_ws
git clone https://github.com/raspberrypi/libcamera.git
git clone https://github.com/raspberrypi/rpicam-apps.git

cd ${HOME}/system_ws/libcamera
meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=enabled \
    -Dgstreamer=enabled \
    -Dtest=true \
    -Dlc-compliance=enabled \
    -Dcam=disabled \
    -Dqcam=enabled \
    -Ddocumentation=disabled \
    -Dpycamera=disabled 
sudo ninja -C build install

cd ${HOME}/system_ws/rpicam-apps
meson setup build --buildtype=release
sudo meson install -C build 
sudo ldconfig
```


