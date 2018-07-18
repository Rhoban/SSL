#!/bin/bash

dir_ssl=$(pwd)

# install all deps
sudo apt update
sudo apt install -y g++ cmake libprotobuf-dev \
    protobuf-compiler php php-cli php-xml \
    libgtest-dev libqt5widgets5 qt5-default \
    libqt5webkit5-dev libboost-dev \
    python-pip python-empy \
    build-essential cmake libqt4-dev \
    libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler \
    libode-dev libboost-dev \
    cmake g++ libgtkmm-2.4-dev libprotobuf-dev protobuf-compiler


###

function install_ssl () {

    cd $dir_ssl

    sudo pip install -U catkin_tools

    ./workspace setup
    ./workspace install

    ./workspace build
}

function install_grsim () {
    cd $dir_ssl/..

    git clone git@github.com:Rhoban/grSim.git
    cd grSim/
    grsim_dir=$(pwd)

    ## Install VarTypes
    cd /tmp
    git clone https://github.com/szi/vartypes.git 
    cd vartypes
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

    ## install grSim
    cd $grsim_dir
    mkdir build
    cd build

    cmake ..
    make
}

function install_referee () {
    cd $dir_ssl/..

    git clone https://github.com/RoboCup-SSL/ssl-refbox.git
    cd ssl-refbox/
    make  
}

install_grsim
install_referee
install_ssl