#!/bin/bash

# array=(watchtower01 watchtower02 watchtower03 watchtower04 watchtower05 watchtower06 watchtower07 watchtower08 watchtower09 watchtower10 watchtower11 watchtower12 watchtower13 watchtower14 watchtower15 )
police_array=(watchtower21 watchtower22 watchtower23 watchtower24 watchtower25 watchtower26 watchtower27 watchtower28 watchtower29 watchtower30 watchtower31 watchtower32 watchtower33 watchtower34 watchtower35 )
montreal_array=(watchtower01 watchtower02 watchtower03 watchtower04 watchtower05 watchtower06 watchtower07 watchtower08 watchtower09 watchtower10 watchtower11 watchtower12 watchtower13 watchtower14 watchtower15 watchtower16)
eth_array=(watchtower41 watchtower42 watchtower43 watchtower44 watchtower45 watchtower46 watchtower47 watchtower48 watchtower49 watchtower50 watchtower51 watchtower52 watchtower53 watchtower54)

array=(${police_array[@]} ${eth_array[@]})

echo $array
echo "We are stopping ${#array[*]} watchtowers"

for index in ${!array[*]}
do
    printf "\nStopping %s\n" ${array[$index]}
    docker -H ${array[$index]}.local stop cslam-acquisition || echo "Didn't stop older cslam-aquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local rm cslam-acquisition || echo "Didn't remove older cslam-aquisition, probably doesn't exist, so don't worry."
done
