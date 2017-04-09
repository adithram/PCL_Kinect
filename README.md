# Installs
PCL

# First build
mkdir build
cd build/
cmake ..

# Fresh build
cd build/
make

# Old CMakeLists.txt
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
