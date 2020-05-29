cd build
cmake ..
make -j2

cd ..

./bin/StereoMatchNCC ./data/filename_left.txt ./data/filename_right.txt
