#!/bin/bash
if [ ! -f /workspace/workspace ]; then
    echo "workspace tool not found! Cloning from its repo..."
    git clone git@github.com:rhoban/workspace /tmp/workspace > /dev/null
    rsync -azP /tmp/workspace/ /workspace > /dev/null
    rm -rf /tmp/workspace
fi

if [ ! -d /workspace/.catkin_tools ]; then
    echo "workspace has not been initialized! Initializing..."
    /workspace/workspace setup

    echo "install rhoban projects & all dependencies"
    /workspace/workspace install git@github.com:rhoban/utils.git
fi

if [ "$1" = 'workspace' ]; then
    exec /workspace/workspace "${@:2}"
elif [ "$#" -eq 0 ]; then
    bash -i
else
    exec "$@"
fi
