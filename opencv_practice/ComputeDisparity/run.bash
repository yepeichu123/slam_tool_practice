cd build/

make -j2

cd ..

./bin/ComputeDisparity ./data/left.png ./data/right.png
