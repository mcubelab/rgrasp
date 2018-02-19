#!/bin/bash
# edit DATA_BASE=$HOME/rgraspdata to your arc data directory

echo "Setting rgrasp environment"

thisFile=$_
if [ $BASH ]
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_CODE_BASE()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export CODE_BASE=$(dirname $configParentDir);;
    "build") export CODE_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: RGRASP environment file is stored in unrecognized location: $thisFile";;
  esac
  export DATA_BASE=$CODE_BASE/../rgraspdata
  export PATH=$PATH:$CODE_BASE/software/build/bin
}

setup_rgrasp()
{
  export PATH=$PATH:$CODE_BASE/software/build/bin
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$CODE_BASE/software/build/lib:$CODE_BASE/software/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$CODE_BASE/software/build/share/java/lcmtypes_arc_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$CODE_BASE/software/build/share/java/drake.jar:$CODE_BASE/software/build/share/java/bot2-lcmgl.jar
  export PKG_CONFIG_PATH=$CODE_BASE/software/build/lib/pkgconfig:$CODE_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH

  # python path
  export PYTHONPATH=$PYTHONPATH:$CODE_BASE/software/build/lib/python2.7/site-packages:$CODE_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"

  export PATH=$PATH:$HOME/software/ffmpeg-2.4.2-64bit-static # for ffmpeg software

  export CUDNN_LIB_DIR=/usr/local/cudnn/v5/lib64  # for arc_vision
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDNN_LIB_DIR


  if [ "$HOSTNAME" == 'mcube-015' ] || [ "$HOSTNAME" == 'mcube-011' ] || [ "$HOSTNAME" == 'rpi-desktop' ] || [ "$HOSTNAME" == 'mcube-014' ] ; then
    export ROS_MASTER_URI=http://mcube-015:11311
  fi
  if [ "$HOSTNAME" == 'mcube-003' ]; then
    export ROS_HOSTNAME=mcube-003
    export ROS_IP=192.168.37.13
  fi
  if [ "$HOSTNAME" == 'mcube-011' ]; then
    export ROS_HOSTNAME=mcube-011
    export ROS_IP=192.168.37.11
  fi
  if [ "$HOSTNAME" == 'mcube-015' ]; then
    export ROS_HOSTNAME=mcube-015
    export ROS_IP=192.168.37.15
  fi
  if [ "$HOSTNAME" == 'mcube-014' ]; then
    export ROS_HOSTNAME=mcube-014
    export ROS_IP=193.168.37.14
  fi
  if [ "$HOSTNAME" == 'rpi-desktop' ]; then
    export ROS_HOSTNAME=rpi-desktop
    export ROS_IP=192.168.37.5
  fi
  if [ "$HOSTNAME" == 'mcube-007' ]; then
    export ROS_HOSTNAME=mcube-007
    export ROS_IP=192.168.37.100
  fi

  export ROSLAUNCH_SSH_UNKNOWN=1
}

setup_clion()
{
    export PATH=$PATH:$HOME/software/clion-2017.1/bin
}

set_ros()
{
  if [ -f $CODE_BASE/catkin_ws/devel/setup.bash ]; then
    source $CODE_BASE/catkin_ws/devel/setup.bash
  else
    source /opt/ros/kinetic/setup.bash
  fi
  export ROS_PACKAGE_PATH=$HOME/rgrasp/ros_ws/:$ROS_PACKAGE_PATH
}

utf8_2_ascii() {
    if [ $# -ne 2 ]; then
        echo "Needs two params. P1: input file, P2: output file"
    else
        cp "$1" "$1.bak"
        konwert utf8-ascii "$1" -o "foo.txt"
        cp foo.txt "$2"
        rm foo.txt
    fi
}

# some useful commands
alias cdrgrasp='cd $CODE_BASE'
alias cdrgraspdata='cd $DATA_BASE'
alias matlabarc='cd $CODE_BASE/software; matlab -nodesktop -nodisplay -nosplash -r "tic; addpath_pods; addpath_drake; toc; cd ../software/planning/ik_server/; ikTrajServerSocket;"'

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $CODE_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot1_SetSpeed 1600 180'
alias faster='rosservice call /robot1_SetSpeed 200 50'
alias fast='rosservice call /robot1_SetSpeed 100 30'
alias slow='rosservice call /robot1_SetSpeed 50 15'

alias gohome='rosservice call robot1_SetJoints "{j1: -0.32913211, j2: -39.41918751, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'
alias goarc='rosservice call robot1_SetJoints   "{j1: -0.08, j2: 12.2, j3: 18.36, j4: 0.0, j5: 59.44, j6: -0.08}"'
alias gohome2='rosservice call robot1_SetJoints "{j1: 0, j2: -40, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'
alias gozero='rosservice call robot1_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0}"'

alias teleop='rosrun teleop teleop'
alias pythonarc='ipython -i -c "run $CODE_BASE/catkin_ws/src/apc_config/python/pythonarc.py"'

alias pman='bot-procman-sheriff -l $CODE_BASE/software/config/rgrasp.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias getjoint='rosservice call robot1_GetJoints'
alias getcart='rosservice call robot1_GetCartesian'
alias setjoint='rosservice call -- robot1_SetJoints'
alias setcart='rosservice call -- robot1_SetCartesian'
alias gocart='rosrun apc_planning go.py'
alias gojoint='rosrun apc_planning go_joint.py'

alias catmake='cd $CODE_BASE/catkin_ws; catkin_make; cd -;'

alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'

alias runarcvirtual='time rosrun arc_planning heuristic.py --jfilename multi_obj_3.json | tee /tmp/$(date +%Y%m%d_%H%M%S)'
alias runarc='time rosrun arc_planning heuristic.py --jfilename arc.json -s -v | tee /tmp/$(date +%Y%m%d_%H%M%S)'
alias rungrasp='rosrun apc_planning grasping17.py'
alias runflush='rosrun apc_planning flush_grasping17.py'
alias profstow='python -m cProfile -o $CODE_BASE/output/planner_prof_$time.txt planner17.py -j item_location_file.json -t '"'"'all\'"'"' -l 5  -d baseline --duration 900'

alias eon='rosservice call /robot1_IOSignal 1 1'
alias eoff='rosservice call /robot1_IOSignal 1 0'
alias son='rosservice call /robot1_IOSignal 2 1'
alias soff='rosservice call /robot1_IOSignal 2 0'
alias pon='rosservice call /robot1_IOSignal 3 1'
alias poff='rosservice call /robot1_IOSignal 3 0'

alias pythonprof='python -m cProfile -o /tmp/tmp.cprof'
alias pythonprofthis='python -m cProfile -o '
alias showprof='pyprof2calltree -k -i /tmp/tmp.cprof'
alias showprofthis='pyprof2calltree -k -i '

alias edg='export EDITOR=geany'

alias resetusb='sudo /home/mcube/arc-vision/scripts/resetUSBports.sh'
alias gripperopen='rosservice call /wsg_50_driver/move 110 50'
alias gripperclose='rosservice call /wsg_50_driver/move 3 50'
alias gripperack='rosservice call /wsg_50_driver/ack'

alias spatulaopen='rosservice call /suction_service "gopen" ""'
alias spatulaclose='rosservice call /suction_service "gclose" ""'

alias testcam='rosrun realsense_camera test_cam.py'
alias patchcam='cd $HOME/software/librealsense/; sudo scripts/patch-realsense-ubuntu-xenial.sh; sudo modprobe uvcvideo'

alias runstowcom='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./stowing_combined_task_run.sh'
alias runpickcom='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./picking_combined_task_run.sh'
alias runpick='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./picking_task_run.sh'
alias runpickfile='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./picking_task_run_with_file.sh'
alias runstow='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./stowing_task_run.sh'

alias runstowcomfile='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./stowing_combined_task_run_with_file.sh'
alias runpickcomfile='cd $HOME/arc/catkin_ws/src/apc_planning/src/; ./picking_combined_task_run_with_file.sh'

alias cdp='cd $HOME/arc/catkin_ws/src/apc_planning/src/ '

alias arcgui='rosrun apc_planning arcgui.py'

alias sshserver='sshpass -p "thecube" ssh mcube@192.168.0.235 -X'
alias sshmain='sshpass -p "thecube" ssh mcube@192.168.0.15 -X'
alias sshfrank='sshpass -p "thecube" ssh mcube@192.168.0.11 -X'
alias sshmaria='sshpass -p "thecube" ssh mcube@192.168.0.10 -X'


ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function ipmasq {
   if [ $# -eq 0 ]; then
     echo 'sharing wlan0 to eth0'
     sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
     sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
   elif [ $# -eq 1 ]; then
     echo "sharing $1 to eth0"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o $1 -j ACCEPT
   elif [ $# -eq 2 ]; then
     echo "sharing $1 to $2"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
   fi
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE

   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cudnn/v5.1/lib64:/usr/local/cuda-8.0/lib64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cudnn/v5.1/lib64:/usr/local/cuda/lib64:/usr/local/cudnn/v5/lib64

if [ -f $HOME/software/torch/install/bin/torch-activate ]; then
  source $HOME/software/torch/install/bin/torch-activate
fi

set_CODE_BASE
setup_rgrasp
set_ros
set_bash
setup_clion
#edg  # use geany to edit

exec "$@" # very important for roslaunch remotely




#################################################
########                             ############
########      Maria's aliases        ############
########                             ############
#################################################

alias sshlab='ssh -X mcube@18.80.1.44 -p 2215'
alias ..='cd ..'
alias c='clear'
alias lg='ls | grep'
alias lpy='ls | grep .py'
# some more ls aliases
alias lla='ls -alF'
alias ll='ls -ltrF'
alias llpy='ls -ltrF | grep .py'
alias llg='ls -ltrF | grep'
alias l.='ls -d .* --color=auto'
alias listalias="alias -p | cut -d= -f1 | cut -d' ' -f2"
alias calculator='bc -l'
alias date='date'
alias meminfo='free -m -l -t'
alias gpull='git pull'
alias gpush='git push'
alias gstatus='git status'
alias h='hisotry'
alias hg='history | grep'
alias restartwifi='sudo service network-manager restart'
alias pyfast='python -m pdb' # from Karpathy's twitter
alias volume='pactl -- set-sink-volume 0 '
# ~/.bashrc
if [[ $- == *i* ]]
then
    bind '"\e[A": history-search-backward'
    bind '"\e[B": history-search-forward'
fi
alias google='firefox google.com'
alias man='manual'
alias findfile='locate'
#alias mostusedcommands='history | awk '{CMD[$4]++;count++;}END { for (a in CMD)print CMD[a] " " CMD[a]/count*100 "% " a;}' | grep -v "./" | column -c3 -s " " -t | sort -nr | nl |  head -n20'
alias github_learning='firefox https://github.com/mcubelab/mcube_learning'
alias github_rgrasp='firefox https://github.com/mcubelab/rgrasp'
alias openfile='xdg-open'
alias gmail='firefox https://mail.google.com/mail/u/0/#inbox'
alias clean_gpu="kill -9  $(nvidia-smi | grep python | sed -n 's/|\s*[0-9]*\s*\([0-9]*\)\s*.*/\1/p' | sort | uniq | sed '/^$/d')"
alias cdlast='cd  "$(\ls -1dt ./*/ | head -n 1)"'
. /usr/share/autojump/autojump.bash
