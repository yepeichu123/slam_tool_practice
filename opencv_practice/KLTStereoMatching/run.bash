cd build
cmake ..
make -j2

cd ..

./build/KLTStereoMatching ./data/left_euroc.png ./data/right_euroc.png
