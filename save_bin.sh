./workspace build
mkdir bin_match3
cp -r bin/* bin_match3/

mkdir bin_match3_debug
./workspace build:debug
cp -r bin/* bin_match3_debug/