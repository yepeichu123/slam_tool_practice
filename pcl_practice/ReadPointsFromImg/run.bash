cd build
make -j2

cd ..
./bin/ReadPointsFromImg ./data/rgb.png ./data/depth.png
