# SSL

## Requirements

### System dependencies

You will need these apt repositories:

    apt-get install g++ cmake libprotobuf-dev \
            protobuf-compiler php php-cli php-xml

### Catkin

Now install catkin:

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

You can then use the following debugging binaries:

* ``./bin/vision``, to display informations from the vision (see ``client/vision.cpp``)
* ``./bin/referee``, to display inforamtions from the referee (see ``client/referee.cpp``)
* ``./bin/sim``, to send commands to the simulator (see ``client/sim.cpp``)

## Packages

Here are the packages:

* ``client``: the package to communicate with the SSL official software (vision, referee and simulator)
    * Note that this provides test binaries in `bin/` directory
* ``viewer``: the viewer to interract with the strategies
* TODO!
