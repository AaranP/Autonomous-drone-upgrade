sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev cmake
sudo apt install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

cd ~/Documents
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false
make -j$(nproc)
sudo make install
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
