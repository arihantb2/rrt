touch .build_incomplete
rm -r build/
mkdir build
cd build
cmake ../
make clean
make -j
cd ..
rm .build_incomplete
echo "Build complete"
