sudo mv /usr/lib/nvidia-384/libEGL.so.1 /usr/lib/nvidia-384/libEGL.so.1.org
sudo mv /usr/lib32/nvidia-384/libEGL.so.1 /usr/lib32/nvidia-384/libEGL.so.1.org
sudo ln -s /usr/lib/nvidia-384/libEGL.so.384.90 /usr/lib/nvidia-384/libEGL.so.1
sudo ln -s /usr/lib32/nvidia-384/libEGL.so.384.90 /usr/lib32/nvidia-384/libEGL.so.1

# also needed to add -DWITH_LAPACK=OFF to cmake command in ./install_opencv.sh