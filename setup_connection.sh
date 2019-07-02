#!/bin/sh

HELP_MESSAGE="Usage: $0 [OPTIONS]

Description:
  setup connection for referre
            
Options:

    -h, --help          Display help
    -e, --eth_name      To choose the eth interface name
    -w, --wlan_name     To choose the wlan interface name
    -r, --restart       Restart network
    
Exemples:

    sudo $0
    sudo $0 --eth_name eth0 --wlan_name wlo1
"


usage() {
    echo "$HELP_MESSAGE"
}


# LOCAL_PATH=$(cd $( dirname $0}) && pwd ) 

# Display usage if the script is not run as root user 
if [ "$USER" != "root" ]; then 
	echo "This script must be run as root!" 
	exit 1
fi 

GET_OPT=`getopt -o  he:w:r --long help,eth_name:wlan_name:restart -n 'setup_connection' -- "$@"`
restart= false

eval set -- "$GET_OPT"
while true
do
    case "${1}" in
        -h|--help)
            usage
            exit 0
            ;;
        -e|--eth_name)
            eth=$2;
            echo "Manual eth = $eth"
            shift 2
            ;;
        -w|--wlan_name)
            eth=$2;
            echo "Manual wlan = $wlan"
            shift 2
            ;;
        -r|--restart)
            restart=true;
            echo "Restart"
            shift
            ;;
        --) 
            shift 
            break
            ;;
        *)  
            echo "Options ${1} is not a known option."
            usage
            exit 1
            ;;
        esac
done

# Automatic search of eth if user didn't specify it
if [ -z "$eth" ]
then
    echo "Automatic search of eth"
    all_eth=`ip a | cut -d":" -f2,2 | grep e | cut -c 2- | grep '^e'`
    eth=$(echo "$all_eth" | head -n 1)
    echo "eth = $eth"
fi

if [ -z "$wlan" ]
then
    echo "Automatic search of wlan"
    all_wlan=`ip a | cut -d":" -f2,2 | grep w | cut -c 2- | grep '^w'`
    wlan=$(echo "$all_wlan" | head -n 1)
    echo "wlan = $wlan"
fi

if [ "$restart" = true ]
then
    sudo ip route del default
    sudo service network-manager restart
    sudo ifconfig $wlan up
    sudo ifconfig $eth up
    sudo ifconfig lo -multicast
else
    sudo ip addr add 10.1.0.52/24 dev lo
    sudo ifconfig $eth down
    sudo ifconfig $wlan down
    sudo ip route del default 
    sudo ip route add default via 10.1.0.1 dev lo
    sudo ifconfig lo multicast
fi