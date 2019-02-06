#!/bin/bash

array=(demowatchtower01 demowatchtower02 demowatchtower03 demowatchtower04 demowatchtower05 demowatchtower06 demowatchtower07 demowatchtower08 demowatchtower09 demowatchtower10 demowatchtower11 demowatchtower12 demowatchtower13 demowatchtower14 demowatchtower15 )

echo "We are stopping ${#array[*]} watchtowers"

for index in ${!array[*]}
do
    printf "\nStopping %s\n" ${array[$index]}
    docker -H ${array[$index]}.local stop cslam-aquisition || echo "Didn't stop older cslam-aquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local rm cslam-aquisition || echo "Didn't remove older cslam-aquisition, probably doesn't exist, so don't worry."
done
