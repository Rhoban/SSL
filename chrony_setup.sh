#!/bin/sh

HELP_MESSAGE="Usage: $0 [OPTIONS]

Description:
  setup chrony
  you can test with: chronyc tracking
            
Options:

    -h, --help          Display help
    
Exemples:

    sudo $0
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

GET_OPT=`getopt -o  h --long help-n 'chrony_setup' -- "$@"`
restart= false

eval set -- "$GET_OPT"
while true
do
    case "${1}" in
        -h|--help)
            usage
            exit 0
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

sudo apt-get install chrony

sudo echo "# Welcome to the chrony configuration file. See chrony.conf(5) for more
# information about usuable directives.

# This will use (up to):
# - 4 sources from ntp.ubuntu.com which some are ipv6 enabled
# - 2 sources from 2.ubuntu.pool.ntp.org which is ipv6 enabled as well
# - 1 source from [01].ubuntu.pool.ntp.org each (ipv4 only atm)
# This means by default, up to 6 dual-stack and up to 2 additional IPv4-only
# sources will be used.
# At the same time it retains some protection against one of the entries being
# down (compare to just using one of the lines). See (LP: #1754358) for the
# discussion.
#
# About using servers from the NTP Pool Project in general see (LP: #104525).
# Approved by Ubuntu Technical Board on 2011-02-08.
# See http://www.pool.ntp.org/join.html for more information.
# pool ntp.ubuntu.com        iburst maxsources 4
# pool 0.ubuntu.pool.ntp.org iburst maxsources 1
# pool 1.ubuntu.pool.ntp.org iburst maxsources 1
# pool 2.ubuntu.pool.ntp.org iburst maxsources 2

server ssl-vision-h.local iburst
# This directive specify the location of the file containing ID/key pairs for
# NTP authentication.
keyfile /etc/chrony/chrony.keys

# This directive specify the file into which chronyd will store the rate
# information.
driftfile /var/lib/chrony/chrony.drift

# Uncomment the following line to turn logging on.
#log tracking measurements statistics

# Log files location.
logdir /var/log/chrony

# Stop bad estimates upsetting machine clock.
maxupdateskew 100.0

# This directive enables kernel synchronisation (every 11 minutes) of the
# real-time clock. Note that it can’t be used along with the 'rtcfile' directive.
rtcsync

# Step the system clock instead of slewing it if the adjustment is larger than
# one second, but only in the first three clock updates.
makestep 1 3
" > /etc/chrony/chrony.conf

sudo service chrony restart

# sudo chronyd -q 'server ssl-vision-h iburst'

# chronyc tracking