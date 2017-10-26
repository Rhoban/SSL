# SSL

## Requirements

### System dependencies

You will need these apt repositories:

    apt-get install g++ cmake libprotobuf-dev \
            protobuf-compiler php php-xml

### Catkin

To install cakin (via pip):

    sudo apt-get install python-pip python-empy
    sudo pip install -U catkin_tools

## Dependencies

After cloning this repository, run:

    ./workspace setup
    ./workspace install

This will install all the dependencies

## Building

To build, run:

    ./workspace build
