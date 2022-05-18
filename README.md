# theta_driver

## Getting started

### with docker

```bash
docker build -t theta_driver .
docker run --rm -it --net=host --privileged theta_driver
ros2 run theta_driver theta_driver_node
```

### without docker

You need to install the libuvc and it's dependencies before using this package.

1. <https://github.com/ricohapi/libuvc-theta>

You will also need the sample for the Theta-Z1

2. <https://github.com/ricohapi/libuvc-theta-sample>

To install the package:

```bash
source /opt/ros/foxy/setup.bash
mkdir -p theta_driver_ws/src
cd theta_driver_ws
git clone -b ros2_foxy https://github.com/JLBicho/theta_driver.git src/theta_driver
git clone https://github.com/stella-cv/libuvc-theta-sample.git src/theta_driver/3rd/libuvc-theta-sample
colcon build
```

Try it with:

```bash
source install/setup.bash
ros2 run theta_driver theta_driver_node 
```

And then you can use image_view package, rqt or rviz2 to see the published image.
