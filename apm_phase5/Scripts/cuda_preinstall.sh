NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5
NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
sudo apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | sudo tail -n +5 > cudasign.pub
sudo echo "$NVIDIA_GPGKEY_SUM cudasign.pub" | sha256sum -c --strict -
rm cudasign.pub
sudo echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list