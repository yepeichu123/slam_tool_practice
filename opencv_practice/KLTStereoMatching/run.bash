mkdir build

cd build
cmake ..
make -j2

cd ..

# trackSingleFrame
#./build/KLTStereoMatching ./data/left_euroc.png ./data/right_euroc.png

# trackEuroc
./build/KLTStereoMatching
