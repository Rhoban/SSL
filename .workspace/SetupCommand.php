<?php

class SetupCommand extends Command
{
    public function getName()
    {
        return 'setup';
    }

    public function getDescription()
    {
        return array('Setup the catkin workspace');
    }

    public function run(array $arguments)
    {
        Terminal::info("* Runing setup of the workspace\n");
        if (!is_dir('src')) {
            mkdir('src');
        }
        if (!is_dir('src/catkin')) {
            Terminal::info("* Cloning catkin\n");
            OS::run('cd src; git clone --depth=1 https://github.com/ros/catkin.git');
            // Patching catkin to get tests working
            OS::run('cd src/catkin/cmake/test/; ln -s ../../python/catkin catkin');
        }
        Terminal::info("* Runnin catkin init\n");
        OS::run('catkin init --workspace .');
        Terminal::info("* Setup of profile debug and release");
        OS::run('catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -msse2"');
        OS::run('catkin config --profile profile -x _profile --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -pg -msse2"');
        OS::run('catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wall -msse2"');
	OS::run('catkin config --profile qtcreator_debug -x _qtdebug --cmake-args -G"CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -msse2"');
    }
}
