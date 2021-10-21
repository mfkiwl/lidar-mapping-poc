build_root=$(pwd)

cd packages
./build.sh

cd $build_root

cd base
make

cd $build_root

cd rover
make

cd src
catkin_init_workspace

cd ..
catkin_make

cd $build_root
