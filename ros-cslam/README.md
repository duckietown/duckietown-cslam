## Diagnostics

To build the image -
```
docker build --file Dockerfile-diagnostics --tag bensonkuan/cslam-diagnostics .
```

To run the image -
```
docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER_URI_DEVICE=Exia -e ROS_MASTER_URI_DEVICE_IP=192.168.1.115 bensonkuan/cslam-diagnostics
```
