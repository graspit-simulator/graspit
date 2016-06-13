if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE 
make -j4 
