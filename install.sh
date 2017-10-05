#/bin/bash
# Peter KT Yu, 2015


function ask {
    echo $1        # add this line
    read -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        return 1;
    else
        exit
        echo "Abort.."
    fi
}

if [ "$#" == 0 ] || [ "$1" == "APT" ]; then
    echo "Install useful packages from apt-get"
    sudo apt-get update
    sudo apt-get --yes install git gitk git-gui geany geany-plugins vim terminator meshlab recordmydesktop meld sagasu openssh-server retext filezilla vlc ipython mesa-utils bmon libyaml-0-2
    sudo apt-get --yes install hardinfo cpufrequtils   # for speedup cpu
        
    sudo apt-get --yes install kcachegrind kcachegrind-converters
    sudo apt-get --yes install build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libportmidi-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev pkg-config protobuf-compiler python-matplotlib libqhull-dev python-pygame doxygen mercurial libglib2.0-dev python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev python-numpy python-scipy python-vtk python-pip libgmp3-dev libblas-dev liblapack-dev libv4l-dev subversion libxmu-dev libusb-1.0-0-dev python-pymodbus graphviz curl libwww-perl libterm-readkey-perl 

    sudo apt-get --yes install libgl1-mesa-dev  # for libbot libGL.so
    # sudo apt-get --yes install virtualbox
    sudo apt-get --yes install compizconfig-settings-manager
    sudo apt-get --yes install libudev-dev  # for realsense
    #sudo pip install --upgrade scipy
    sudo pip install chan 
    sudo pip install openpyxl
    sudo pip install sklearn
    sudo pip install plyfile
    sudo pip install tabulate
    sudo pip install ipdb
    sudo pip install pyprof2calltree
    sudo pip install tabulate
    sudo pip install gTTs
    sudo pip install python-vlc
    sudo pip install --upgrade requests
    sudo pip install enum34
    sudo easy_install pycollada
    sudo apt-get --yes install byacc flex
    sudo apt-get --yes install libcgal-dev # for apriltags from personalrobotics
fi

if [ "$#" == 0 ] || [ "$1" == "LABJACK" ]; then
  mkdir -p ~/Downloads/exodriver
  wget -P ~/Downloads -q https://labjack.com/sites/default/files/2015/05/LabJackPython-5-26-2015.zip
  wget -P ~/Downloads/exodriver -q https://github.com/labjack/exodriver/archive/v2.5.3.zip
  #wget -P ~/Downloads -q https://github.com/labjack/exodriver/tarball/master
  mkdir -p $HOME/software
  unzip -o ~/Downloads/exodriver/v2.5.3.zip -d $HOME/software
  cd $HOME/software/exodriver-2.5.3
  sudo ./install.sh
  unzip -o ~/Downloads/LabJackPython-5-26-2015.zip -d $HOME/software
  cd $HOME/software/LabJackPython-5-26-2015
  sudo python setup.py install
fi

if [ "$#" == 0 ] || [ "$1" == "LCM" ]; then
  # install LCM
  wget -P ~/Downloads -q https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
  mkdir -p $HOME/software
  unzip -o ~/Downloads/lcm-1.3.1.zip -d $HOME/software
  
  cd $HOME/software/lcm-1.3.1
  ./bootstrap.sh
  ./configure
  make
  sudo make install
fi

if [ "$#" == 0 ] || [ "$1" == "LIBBOT" ]; then
  cd $HOME/software
  git clone git@github.com:mcubelab/libbot.git
  cd libbot
  sudo make BUILD_PREFIX=/usr/local
fi

if [ "$#" == 0 ] || [ "$1" == "ROSK" ]; then
    echo "Install ROS Kinetic"

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get --yes install ros-kinetic-desktop-full
    sudo rosdep init
    rosdep update
    sudo apt-get --yes install python-rosinstall
    source /opt/ros/kinetic/setup.bash

    sudo apt-get --yes install ros-kinetic-moveit
    sudo apt install --yes ros-kinetic-pcl-conversions
    sudo apt install --yes ros-kinetic-pcl-ros
    sudo apt-get --yes install ros-kinetic-joy
    sudo apt-get --yes install ros-kinetic-perception  # for cv_bridge

#    sudo cp joint_state_publisher/joint_state_publisher /opt/ros/kinetic/lib/joint_state_publisher/joint_state_publisher
fi

if [ "$#" == 0 ] || [ "$1" == "CATKIN" ]; then 
    git submodule update --init --recursive catkin_ws/src/pr_msgs
    echo "Make CATKIN"
    cd $CODE_BASE/catkin_ws
    catkin_make
fi

if [ "$#" == 0 ] || [ "$1" == "ABB" ]; then
    echo "Make abb-ros"
    git submodule update --init --recursive catkin_ws/abb-ros-catkin
    cd $CODE_BASE/catkin_ws
    catkin_make
    source $CODE_BASE/software/config/arc_environment.sh
fi

#if [ "$#" == 0 ] || [ "$1" == "SOFTWARE" ]; then
#    echo "Make SOFTWARE"
#    git submodule update --init --recursive software/externals/snopt software/externals/drake
#    cd $CODE_BASE/software/
#    make -j
#fi

if [ "$#" == 0 ] || [ "$1" == "HAND" ]; then 
    git submodule update --init --recursive catkin_ws/src/wsg50-ros-pkg
    echo 'Do catkin_make --pkg wsg_50_common wsg_50_driver wsg_50_simulation'
    cd $CODE_BASE/catkin_ws
    catkin_make 
    source $CODE_BASE/software/config/arc_environment.sh
    catkin_make 
fi

if [ "$#" == 1 ] && [ "$1" == "CLION" ]; then 
    cd $HOME/Downloads
    wget -q https://download.jetbrains.com/cpp/CLion-2017.1.tar.gz
    mkdir -p $HOME/software
    tar -xzvf CLion-2017.1.tar.gz -C $HOME/software
    echo 'Installing CLion'
    cd $HOME/software/clion-2017.1/bin
    ./clion.sh
fi


if [ "$#" == 0 ] || [ "$1" == "LCM" ]; then
    # install LCM
    wget -P ~/Downloads -q https://github.com/lcm-proj/lcm/archive/v1.3.1.tar.gz
    
    mkdir -p $HOME/software
    cp ~/Downloads/lcm-1.3.1.tar.gz $HOME/software
    rm ~/Downloads/lcm-1.3.1.tar.gz
              
    cd $HOME/software/
    tar -xzvf lcm-1.3.1.tar.gz
    cd lcm-1.3.1/
    ./bootstrap.sh
    ./configure
    make
    sudo make install
    cd ..
    rm lcm-1.3.1.tar.gz
fi
                  
if [ "$#" == 0 ] || [ "$1" == "LIBBOT" ]; then
    mkdir -p $HOME/software
    cd $HOME/software
    git clone https://github.com/mit212/libbot.git
    cd libbot
    sudo make -j
    sudo cp -r ../build/* /usr/local/  # some weird situation for VM ware
    sudo cp -r build/* /usr/local/
    echo "Ignore a cannot copy error just above, it tries 2 possible locations and 1 always fails"
fi

# sudo apt-get remove --purge cuda-*
# sudo apt-get remove --purge nvidia-*
# sudo apt-get remove --purge nvidia-*
if [ "$#" == 1 ] && [ "$1" == "NV367" ]; then # TODO CHECK IF NEEDED
    wget -P ~/Downloads http://web.mit.edu/peterkty/www/shared/nvidiadriver/NVIDIA-Linux-x86_64-367.18.run
    chmod +x ~/Downloads/NVIDIA-Linux-x86_64-367.18.run
    sudo ~/Downloads/NVIDIA-Linux-x86_64-367.18.run
fi

if [ "$#" == 1 ] && [ "$1" == "NV375" ]; then # TODO CHECK IF NEEDED
   sudo apt-get purge nvidia-*
   sudo add-apt-repository ppa:graphics-drivers/ppa
   sudo apt-get update
   sudo apt-get install nvidia-375
fi

if [ "$#" == 1 ] && [ "$1" == "CUDA8" ] || [ "$1" == "VISION" ]; then
    wget -P ~/Downloads https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run 
    cd ~/Downloads
    echo "Please download cuda_8.0.27_linux.run to ~/Downloads/"
    chmod +x ~/Downloads/cuda_8.0.61_375.26_linux-run
    echo "When prompted, say no to the drivers install"
    echo "Make sure you are in arc env"
    sudo ~/Downloads/cuda_8.0.61_375.26_linux-run
fi

if [ "$#" == 1 ] && [ "$1" == "CUDNN8" ] || [ "$1" == "VISION" ]; then
    cd ~/Downloads/
    wget -q http://web.mit.edu/peterkty/www/shared/cudnn/cudnn-8.0-linux-x64-v5.0-ga.tgz
    tar -zxvf cudnn-8.0-linux-x64-v5.0-ga.tgz
    CUDNN_LIB_DIR=/usr/local/cudnn/v5/lib64
    sudo mkdir -p $CUDNN_LIB_DIR/
    sudo mkdir -p /usr/local/cudnn/v5/include/
    
    sudo cp cuda/lib64/* $CUDNN_LIB_DIR/
    sudo cp cuda/include/* /usr/local/cudnn/v5/include/
    cd -
fi
    
if [ "$#" == 1 ] && [ "$1" == "OPENCV" ] || [ "$1" == "VISION" ]; then
    sudo apt-get install libopencv-dev
fi

if [ "$#" == 1 ] && [ "$1" == "REALSENSE" ] || [ "$1" == "VISION" ]; then
    echo "Install librealsense"
    mkdir -p $HOME/software
    cd $HOME/software
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    sudo apt-get update
    sudo apt-get install --assume-yes libusb-1.0-0-dev pkg-config
    sudo apt-get install --assume-yes libglfw3-dev
    sudo apt-get install --assume-yes libssl-dev
    mkdir build
    cd build
    cmake ..
    make -j && sudo make install
    sudo ldconfig
    cd ..
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger
    ./scripts/patch-realsense-ubuntu-xenial.sh
    sudo modprobe uvcvideo
    sudo dmesg | tail -n 50
fi
if [ "$#" == 1 ] && [ "$1" == "TORCH" ] || [ "$1" == "VISION" ]; then
    echo "Install TORCH"
    cd $HOME/software
    git clone https://github.com/torch/distro.git torch --recursive
    cd torch
    sudo bash install-deps
    ./install.sh
    source $HOME/.bashrc
    sudo ldconfig
    sudo apt-get install luarocks
    sudo luarocks --from=https://raw.githubusercontent.com/torch/rocks/master/ install inn
    sudo apt-get install libhdf5-serial-dev hdf5-tools
    cd $HOME/software
    git clone https://github.com/deepmind/torch-hdf5
    cd torch-hdf5
    sudo $HOME/software/torch/install/bin/luarocks install totem
    sudo $HOME/software/torch/install/bin/luarocks make hdf5-0-0.rockspec LIBHDF5_LIBDIR="/usr/lib/x86_64-linux-gnu/"
fi

if [ "$#" == 1 ] && [ "$1" == "PCL" ]; then
    echo "Install PCL"
    sudo apt-get install --assume-yes git build-essential linux-libc-dev
    sudo apt-get install --assume-yes cmake cmake-gui 
    sudo apt-get install --assume-yes libusb-1.0-0-dev libusb-dev libudev-dev
    sudo apt-get install --assume-yes mpi-default-dev openmpi-bin openmpi-common  
    sudo apt-get install --assume-yes libflann1.8 libflann-dev
    sudo apt-get install --assume-yes libeigen3-dev
    sudo apt-get install --assume-yes libboost-all-dev
    sudo apt-get install --assume-yes libqhull* libgtest-dev
    sudo apt-get install --assume-yes freeglut3-dev pkg-config
    sudo apt-get install --assume-yes libxmu-dev libxi-dev 
    sudo apt-get install --assume-yes mono-complete
    sudo apt-get install --assume-yes qt-sdk openjdk-8-jdk openjdk-8-jre
fi

if [ "$#" == 1 ] && [ "$1" == "VISIONMODULES" ] || [ "$1" == "VISION" ]; then
    cd $CODE_BASE/catkin_ws/src
    git submodule update --init --recursive realsense_camera
    git submodule update --init --recursive passive_vision
    git submodule update --init --recursive active_vision
    cd $CODE_BASE/catkin_ws
    catkin_make --pkg realsense_camera
fi

if [ "$#" == 1 ] && [ "$1" == "MATLAB" ]; then
    wget -q http://esd.mathworks.com/R2017b/Linux_x86_64/INST_434233/matlab_R2017a_glnxa64.zip
    echo "Please download matlab to ~/Downloads/matlab_R2016a_glnxa64.zip"
    ask "Ready? [y/N]" 
    cd ~/Downloads/
    unzip -o matlab_R2017a_glnxa64.zip -d matlab_R2017a_glnxa64
    cd matlab_R2017a_glnxa64
    echo "In the install wizard, install the matlab to your home folder e.g. $HOME/MATLAB/R2016a"
    ./install
    
    sudo rm /usr/local/bin/matlab
    sudo rm /usr/local/bin/mbuild
    sudo rm /usr/local/bin/mcc
    sudo rm /usr/local/bin/mex
    sudo ln -s $HOME/MATLAB/R2017a/bin/matlab /usr/local/bin/matlab
    sudo ln -s $HOME/MATLAB/R2017a/bin/mbuild /usr/local/bin/mbuild
    sudo ln -s $HOME/MATLAB/R2017a/bin/mcc /usr/local/bin/mcc
    sudo ln -s $HOME/MATLAB/R2017a/bin/mex /usr/local/bin/mex
    
    sudo ln -s $HOME/MATLAB/R2017a_light/bin/matlab /usr/local/bin/matlab_light
fi
