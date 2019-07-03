# SSL - AI

## Development Guidelines

The code follows the guideline describe in ROS. [ROS_Guideline](http://wiki.ros.org/CppStyleGuide)

### Format

We using a .clang-format file. Caution ! Use the clang-format version 6.0, if your OS doesn't install it automatically then install clang-format-6.0 and rename the command to use the format script.

## Requirements

### System dependencies

You will need theses packets:

``` bash
sudo apt-get install -y g++ cmake libprotobuf-dev \
  protobuf-compiler php php-cli php-xml \
  libgtest-dev libwebsockets-dev gnuplot
```

### Catkin

Now install catkin:

``` bash
sudo apt-get install -y python-pip python-empy
# Eventually, if necessary, install python3-trollius
sudo apt-get install -y python3-trollius
sudo pip install -U catkin_tools
```

## Dependencies

After cloning this repository, run:

``` bash
./workspace setup
./workspace install
```

This will install all the dependencies.

See also clang-format above.

## Building

To build, run:

``` bash
./workspace build
```

Then, you can use the following debugging binaries:

* `./bin/vision`, to display informations from the vision (see `client/vision.cpp`)
* `./bin/referee`, to display informations from the referee (see `client/referee.cpp`)
* `./bin/sim`, to send commands to the simulator (see `client/sim.cpp`)

## Documentation

From workspace root:

``` bash
cd doc
mkdir build
cd build
cmake ..
make doc
```

Documentation is inside html directory.

## Testing

Tests can be built using the command: `./workspace tests`

To run the tests (once built), use the command: `./workspace test`

Access to the test message is controlled by adding the flag `--verbose`.

Example: Building and showing tests, but only for ssl_ai
`./workspace tests --verbose --no-deps ssl_ai`

## Packages

Here are the packages:

* `client`: the package to communicate with the SSL official software (vision, referee and simulator)
  * Note that this provides test binaries in `bin/` directory
* `viewer`: the viewer to interact with the strategies
* TODO!

## QtCreator

To use QtCreator, you have to add following line at the end of the file .workspace/SetupCommand.php:

below the line:

``` php
OS::run('catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wall -msse2"');
```

add the two lines:

``` php
OS::run('catkin config --profile qtcreator_debug -x _qtdebug --cmake-args -G"CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -msse2"');

OS::run('catkin config --profile qtcreator_release -x _qtrelease --cmake-args -G"CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wall -msse2"');
```

This will create a debug and release profile with CodeBlocks support (needed by QtCreator).

Then:

``` bash
./workspace setup
./workspace build --profile=qtcreator_debug
./workspace build --profile=qtcreator_release
```

Now, you can load a CMakeLists.txt into QtCreator:

``` bash
qtcreator src/ai/CMakeLists.txt
```

In the configure project window, uncheck desktop and choose "import compil from..." and select `build_qtdebug/ssl_ai` then click on Import button. This will create a temporary imported kit with only one checked. You can add various setup (debug/release/...) by selecting the right building directory (i.e. build_qtrelease/ssl_ai).

Finally, click on "Configure Project" button.

If you want to reset your Qt configuration, just remove the file CMakeLists.txt.user. Also, please never add this file to the git repository.
