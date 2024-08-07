#!/usr/bin/env bash
current_dir=$(pwd)

# Define the directory you want to switch to (replace with your desired path)
target_dir="/opt/intel/openvino/"

# Print a message indicating the switch
echo "Switching to directory: $target_dir"

# Change directory to the target directory
cd "$target_dir"

# Perform any actions you want to execute in the target directory (optional)
source setupvars.sh
# Print a message indicating the return
echo "Returning to original directory"

# Change directory back to the original directory stored in the variable
cd "$current_dir"

echo "Configuring and building Thirdparty libraries ..."
echo "BoW2 ..."

cd Thirdparty
cd DBoW2
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ../..


echo "BoW2 ..."
cd g2o
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ../..

echo "Osmap ..."
cd Osmap
sh build.sh
cd ../..

echo "Extracting vocabulary file ..."
cd Vocabulary
tar -xvf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building OA-SLAM ..."
mkdir build
cd build
cmake ..
make -j$(nproc)
