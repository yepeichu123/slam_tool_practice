mkdir build
cd build
cmake ..
make -j2

cd ..
./bin/UndistorAndRemap ./data/MH_05_difficult-left.txt ./data/MH_05_difficult-right.txt
