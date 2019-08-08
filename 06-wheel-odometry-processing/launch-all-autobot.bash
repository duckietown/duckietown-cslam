
for i in 03; do
    docker-compose run -d -e ACQ_DEVICE_NAME=autobot$i --name odometryprocessor$i odometry-processor;
done
