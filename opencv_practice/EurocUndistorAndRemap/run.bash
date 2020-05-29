cd build
cmake ..
make -j2

cd ..
./bin/UndistorAndRemap ./data/MH_01_easy-left.txt ./data/MH_01_easy-right.txt 
