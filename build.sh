touch .build_incomplete
rm -r build/
mkdir build
cd build
cmake ../
make clean
make -j
make test
cd ..
rm .build_incomplete
echo "Build complete"
