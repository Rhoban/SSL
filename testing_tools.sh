#!/bin/bash

ai=2
interface=''

while [[ "$1" =~ ^- && ! "$1" == "--" ]]; do 

    case $1 in
        -a | --ai )
            shift;
            ai=$1
        ;;
        -i | --interface )
            shift;
            interface=$1
        ;;
        -h | --help )
            help='true'
        ;;
    esac; 
    shift; 
done

if [[ -z "$interface" ]]; then
    help='true'
fi


if [ -n "$help" ]; then
    echo '--- USAGE ---'
    echo '[ -i || --interface ] { internet_interface }'
    echo '[ -a || --ai ] { numbers_of_ai }'
    exit 1
fi

# config network
sudo ifconfig $interface 10.0.0.1 netmask 255.255.255.0
sudo route del default
sudo route add default gw 10.0.0.254
sudo ifup $interface
sudo systemctl restart networking

# grSim
rm ~/.grsim.xml
cd ../grSim/
./bin/grsim --headless &

# referee
cd ../ssl-refbox/
./sslrefbox &

# AI
cd ../SSL/

./bin/viewer -s -y -p 10020 -t AMC &

if [[ $ai == 2 ]]; then
    ./bin/viewer -s -p 10020 -t toto &
fi


## AI Blue

exit_executed=false

exit_children() {
    sudo route del default
    sudo ip addr flush $interface
    sudo ifdown $interface
    sudo systemctl restart networking
    sudo ifup  $interface
    # sudo dhclient
    kill $(ps -o pid= --ppid $$)
    exit_executed=true
}

trap exit_children SIGINT SIGTERM

wait

if [[ exit_executed == false ]]; then
    exit_children
fi

exit 0