#!/usr/bin/env bash

DEPS_DIR=false
CATKIN_DIR=false
WARNCOLOR='\033[1;36m'
NC='\033[0m' # No Color
PREFIX="${WARNCOLOR}INSTALLER:"

instlog() {
    printf "$PREFIX $@ $NC\n"
}

while [ "$#" -gt 0 ]; do
  case $1 in
    -h) printf "usage: $0 \n
    [-c] catkin_directory (Recommend ~/repos/sub_dependencies)\n
    [-d] dependencies_directory  (Recommend ~/repos/catkin_ws)\n
    example: ./install.sh -d ~/repos/sub_dependencies -c ~/repos/catkin_ws
    ..."; exit ;;
    # TODO: Use this to check if catkin ws already set up
    -c) CATKIN_DIR="$2"
        shift 2;;
    -d) DEPS_DIR="$2"
        shift 2;;
    -?) echo "error: option -$OPTARG is not implemented"; exit ;;
  esac
done

# Spooky activity to check if we are in semaphore
if ls ~/ | grep --quiet -i Sub8$; then
    DEPS_DIR=~/repos/sub_dependencies;
    CATKIN_DIR=~/repos/catkin_ws;
fi

if ! [ -n $DEPS_DIR ]; then
    instlog "Please set a directory to install dependencies using ./install.sh -d /my/directory/"
    exit
fi

if ! [ -n $CATKIN_DIR ]; then
    instlog "Please set a directory to install catkin stuff using ./install.sh -c /my/catkin_ws/"
    instlog "If you have an existing catkin workspace, it is important that you specify it"
    exit
fi

instlog "Installing dependencies in $DEPS_DIR"
instlog "Generating catkin workspace (If needed) and installing ROS dependencies at $CATKIN_DIR"

CATKIN_WS_ALREADY_EXISTS=false;
## Can't get this to work properly.
# if source /opt/ros/indigo/setup.bash; then
#    roscd; cd ..
#    if ! echo $PWD | grep --quiet ^/opt; then
#        CATKIN_WS_ALREADY_EXISTS=true;
#        CATKIN_DIR=$PWD
#        echo "Found existing catkin workspace at $CATKIN_DIR"
#    else
#        echo "PGenerating catkin workspace (If needed) and installing dependencies at $CATKIN_DIR"
#    fi
# else
#     echo "Generating catkin workspace (If needed) and installing dependencies at $CATKIN_DIR"
# fi
# TODO: Force this to be run as root, and manually run the non-root commands as w/e
#if (( $EUID != 0 )); then
#    echo "Error: You are not root. Please run 'install.sh' instead"
#    exit
#fi

mkdir -p "$DEPS_DIR"
cd "$DEPS_DIR"
####### Always Pre-requisites

sudo add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) main universe"
if ! cat /etc/apt/sources.list | grep --quiet "^[^#;]*deb-src .*. trusty main universe$"; then
    sudo sh -c 'echo "deb-src http://archive.ubuntu.com/ubuntu trusty main universe" >> /etc/apt/sources.list'
fi
if ! $CATKIN_WS_ALREADY_EXISTS; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    instlog "Adding key -- if this fails, try commenting out the line"
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
fi
instlog "Updating apt-get"
sudo apt-get update -qq

instlog "Getting build stuff"
sudo apt-get install -qq cmake python-pip

####### Make tools
instlog "Getting misc make tools"
sudo apt-get install -qq binutils-dev

####### Python stuff
# Deal with pyODE
# TODO: Come up with a way to test that pyode is both installed and fixed
instlog "Fixing Pyode"
sudo apt-get install -qq python-pyode
sudo rm -fr /tmp/pyode-build
sudo mkdir -p /tmp/pyode-build
cd /tmp/pyode-build
sudo apt-get build-dep -qq -y python-pyode
sudo apt-get remove -qq -y python-pyode
sudo apt-get source -qq --compile python-pyode
sudo dpkg -i python-pyode_*.deb 2>&1 >/dev/null

instlog "Installing setuptools"
sudo pip install -q -U setuptools

# Normal things
instlog "Getting miscellaneous libraries"
sudo apt-get install -qq libboost-all-dev python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial

####### Vispy
# Check if Vispy is installed
cd "$DEPS_DIR"
pip freeze | grep --quiet -i vispy
if [ $? -eq 1 ]; then

    instlog "Looks like you don't have Vispy, let's grab it"
    # Let's force ourselves to stay at the latest version
    # TODO: python -c "import vispy; print vispy.__version__" -> Compare to what's available
    # TODO: Uninstall old Vispy if it is out of date
    git clone -q https://github.com/vispy/vispy.git
    cd vispy
    sudo python setup.py develop
else
    instlog "You have Vispy, don't worry, we'll make sure it's up to date"
    # How's that for a hack!? (Figure out location of Vispy)
    cd "$(dirname `python -c  "import vispy; print vispy.__file__"`)/.."
    sudo python setup.py develop --uninstall
    if [ $? -eq 1 ]; then
        instlog "Your Vispy installation is weird, don't install in manually, use the the sub install script"
    fi
    git pull
    sudo python setup.py develop
fi
####### End Vispy

####### Ceres
cd "$DEPS_DIR"
# TODO: Make this better (It might not be installed in /usr/local!)
# ls /usr/local/share/ | grep -i ceres
if ! ls /usr/local/share/ | grep --quiet -i ceres$; then
    instlog "Looks like to don't have Google Ceres, we'll install it"
    sudo apt-get -qq install libgoogle-glog-dev
    # BLAS & LAPACK
    sudo apt-get -qq install libatlas-base-dev
    # Eigen3
    sudo apt-get -qq install libeigen3-dev
    # SuiteSparse and CXSparse (optional)
    # - If you want to build Ceres as a *static* library (the default)
    #   you can use the SuiteSparse package in the main Ubuntu package
    #   repository:
    sudo apt-get -qq install libsuitesparse-dev

    wget http://ceres-solver.org/ceres-solver-1.11.0.tar.gz
    # Unzip
    tar zxf ceres-solver-1.11.0.tar.gz
    # Delete the zip trash
    rm ./ceres-solver-1.11.0.tar.gz
    mkdir ceres-bin
    cd ceres-bin
    cmake ../ceres-solver-1.11.0
    make -j3
    sudo make install
fi

cd "$DEPS_DIR"

# TODO: rm -rf ./ceres-solver-1.11.0
####### End Ceres

####### Install ROS
if which rosrun; then
    instlog "Nice, you already have ROS. We're lazy, so we didn't check the version. Just promise you have Indigo"
else
    instlog "Looks like ROS is not installed, let's take care of that"
    sudo apt-get install -qq ros-indigo-desktop python-catkin-pkg python-rosdep
    source /opt/ros/indigo/setup.bash
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    sudo rosdep init
    rosdep update
fi


# ROS!
mkdir -p "$CATKIN_DIR/src"
####### Install the ROS packages that we use
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq ros-indigo-sophus

####### SBA
cd "$CATKIN_DIR/src"
# Get Jake's fork of BPCG (Minor changes to make it build)
if rospack list | grep --quiet bpcg$; then
    instlog "You have Block-PCG installed, not installing!"
else
    instlog "Cloning Jake's fork of BPCG"
    git clone -q https://github.com/jpanikulam/vslam-bpcg.git
fi

# Get Jake's fork of sba (Minor changes to make it build)
if rospack list | grep --quiet sba$; then
    instlog "You have sba installed, not installing!"
else
    instlog "Cloning Jake's fork of SBA"
    git clone -q https://github.com/jpanikulam/sba.git
fi
####### End SBA

####### Check if the sub is set up, and if it isn't, set it up
if ! ls "$CATKIN_DIR"/devel | grep --quiet setup.bash$; then
    instlog "Looks like you don't have a catkin workspace set up at $CATKIN_DIR, making one"
    cd "$CATKIN_DIR/src"
    catkin_init_workspace
    catkin_make -C "$CATKIN_DIR"
    echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
else
    instlog "Looks like you already have a catkin workspace set up where you specified"
fi

source "$CATKIN_DIR/devel/setup.bash"

if ! ls "$CATKIN_DIR/src" | grep --quiet Sub8$; then
    instlog "Looks like you don't have the sub set up, let's do that"
    # Make our sub stuff
    # Check if Sub8 repository already exists (Semaphore puts Sub8 in ~/Sub8)
    instlog "Looking for Sub8 (Are we in Semaphore?)"
    if  ls ~/ | grep --quiet -i Sub8$; then
        # Here, we assume we're in Semaphore-ci, and should run catkin_make in the actual build thread
        instlog "Found Sub8 in HOME, Assuming we're in Semaphore"
        mv ~/Sub8 "$CATKIN_DIR/src"
    else
        if ! ls "$CATKIN_DIR/src" | grep --quiet Sub8$; then
            instlog "Cloning the sub"
            cd "$CATKIN_DIR/src"
            git clone -q https://github.com/uf-mil/Sub8.git
            cd Sub8
            git remote rename origin upstream
            instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
            catkin_make -C "$CATKIN_DIR"
        fi
    fi
else
    echo "You already have the Sub ready! Now you cando anything you want!"
    catkin_make -C "$CATKIN_DIR"
fi
