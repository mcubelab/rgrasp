#/bin/bash
# Eudald Romo Grau, 2017


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

if [ "$#" == 0 ] || [ "$1" == "MSGS" ]; then
    echo "Make abb-ros"
    git submodule update --init --recursive catkin_ws/src/pr_msgs
fi

if [ "$#" == 0 ] || [ "$1" == "ABB" ]; then
    echo "Make abb-ros"
    git submodule update --init --recursive catkin_ws/src/abb-ros-catkin
fi

if [ "$#" == 0 ] || [ "$1" == "HAND" ]; then 
    git submodule update --init --recursive catkin_ws/src/wsg50-ros-pkg
    echo 'Do catkin_make --pkg wsg_50_common wsg_50_driver wsg_50_simulation'
    cd $CODE_BASE/catkin_ws
    catkin_make --pkg wsg_50_common wsg_50_driver wsg_50_simulation
    source $CODE_BASE/software/config/rgrasp_environment.sh
    catkin_make 
fi

if [ "$#" == 0 ] || [ "$1" == "WEIGHT" ]; then
    git submodule update --init --recursive catkin_ws/src/weight_sensor
    catkin_make
fi
    

if [ "$#" == 1 ] && [ "$1" == "VISION" ]; then
    git submodule update --init --recursive catkin_ws/src/passive_vision
    git submodule update --init --recursive catkin_ws/src/active_vision
    git submodule update --init --recursive catkin_ws/src/realsense_camera
fi
