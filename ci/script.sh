if [ "$TRAVIS_PULL_REQUEST" = "false" ] && [ "$COMPILER" = "CLANG" ]; then exit; fi

git clone git@github.com:google/googletest.git
cd googletest
git checkout release-1.8.0
mkdir build && cd build 
cmake ..
make -j4
sudo make install

cd ..
cd ..

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE  -DBUILD_TESTS:BOOL=true
make -j4 
