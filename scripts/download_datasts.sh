function download_tum_dataset() {
  if [ -e "$TUM_DIR/$1" ]; then
    echo "The path $TUM_DIR/$1 already exists, skipping..."
  else
    if [[ "dataset-room1_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room1_512_16.bag
      if [ $? -ne 0 ]; then 
        rm -f ./dataset-room1_512_16.bag
      else
        convert_rosbag_to_ros2 './dataset-room1_512_16.bag' './room1'
        mv dataset-room1_512_16.bag room1 && mv room1/dataset-room1_512_16.bag room1/room1.bag
      fi
    elif [[ "dataset-room2_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && mkdir room2 && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room2_512_16.bag
      convert_rosbag_to_ros2 './dataset-room2_512_16.bag' './'
    elif [[ "dataset-room3_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && mkdir room3 && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room3_512_16.bag
      convert_rosbag_to_ros2 './dataset-room3_512_16.bag' './'
    elif [[ "dataset-room4_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && mkdir room4 && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag
      convert_rosbag_to_ros2 './dataset-room4_512_16.bag' './'
    elif [[ "dataset-room5_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && mkdir room5 && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room5_512_16.bag
      convert_rosbag_to_ros2 './dataset-room5_512_16.bag' './'
    elif [[ "dataset-room6_512_16" == *"$1"* ]]; then
      cd $TUM_DIR && mkdir room6 && wget http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room6_512_16.bag
      convert_rosbag_to_ros2 './dataset-room6_512_16.bag' './'
    fi
  fi
}

function convert_rosbag_to_ros2() {
  rosbags-convert $1 --dst $2
}

function download_euroc_dataset() {
  if [ -e "$EUROC_DIR/$1" ]; then
    echo "The path $EUROC_DIR/$1 already exists, skipping..."
  else
  
    if [ "$1" == "MH_01" ]; then
      cd $EUROC_DIR && gdown 1cbVXoOzsqOEkwCGAVw7QwP-npp2S_W0G
      cd $EUROC_DIR && mkdir MH_01 && unzip -j MH_01_easy.zip -d MH_01 && rm -f MH_01_easy.zip 
    elif [ "$1" == "MH_02" ]; then
      cd $EUROC_DIR && gdown 18-H4MJHabvuaVdfcreShOtAKf88kusqB
      cd $EUROC_DIR && mkdir MH_02 && unzip -j MH_02_easy.zip -d MH_02 && rm -f MH_02_easy.zip 
    elif [ "$1" == "MH_03" ]; then
      cd $EUROC_DIR && gdown 1cpueLI-XDHc_EpauNrgaw7K0-Y3Mv-r0
      cd $EUROC_DIR && mkdir MH_03 && unzip -j MH_03_medium.zip -d MH_03 && rm -f MH_03_medium.zip 
    elif [ "$1" == "MH_04" ]; then
      cd $EUROC_DIR && gdown 15dKMBC1pRX0ZyUpxKmH9u0Jqxex43yFO
      cd $EUROC_DIR && mkdir MH_04 && unzip -j MH_04_difficult.zip -d MH_04 && rm -f MH_04_difficult.zip 
    elif [ "$1" == "MH_05" ]; then
      cd $EUROC_DIR && gdown 1dLtZ_Mapo6CGUMdXskbnFGaf2maNKWQL
      cd $EUROC_DIR && mkdir MH_05 && unzip -j MH_05_difficult.zip -d MH_05 && rm -f MH_05_difficult.zip 
    # V1
    elif [ "$1" == "V1_01" ]; then
      cd $EUROC_DIR && gdown 1b4MRugEWNB9XEUtAz3gdHEpYn4fBO0Tv
      cd $EUROC_DIR && mkdir V1_01 && unzip -j V1_01_easy.zip -d V1_01 && rm -f V1_01_easy.zip 
    elif [ "$1" == "V1_02" ]; then
      cd $EUROC_DIR && gdown 1qSmq2CRloOMv540bh94qrm-susdMLhbo
      cd $EUROC_DIR && mkdir V1_02 && unzip -j V1_02_medium.zip -d V1_02 && rm -f V1_02_medium.zip 
    elif [ "$1" == "V1_03" ]; then
      cd $EUROC_DIR && gdown 12OQ5nAyEHCJ8AXbjzEX5_Hw2-jwyliyN
      cd $EUROC_DIR && mkdir V1_03 && unzip -j V1_03_difficult.zip -d V1_03 && rm -f V1_03_difficult.zip 
    # V2
    elif [ "$1" == "V2_01" ]; then
      cd $EUROC_DIR && gdown 1VEPA-ouCLgcVZ2FXiwD8N-RgA1LknDqh
      cd $EUROC_DIR && mkdir V2_01 && unzip -j V2_01_easy.zip -d V2_01 && rm -f V2_01_easy.zip 
    elif [ "$1" == "V2_02" ]; then
      cd $EUROC_DIR && gdown 1S1Wn--8AW_ip6bnZv3BiDydbO_xzZTf4
      cd $EUROC_DIR && mkdir V2_02 && unzip -j V2_02_medium.zip -d V2_02 && rm -f V2_02_medium.zip 
    elif [ "$1" == "V2_03" ]; then
      cd $EUROC_DIR && gdown 1KIvnh001M5pmSU7fTtfvAKWBTolFFVW0
      cd $EUROC_DIR && mkdir V2_03 && unzip -j V2_03_difficult.zip -d V2_03 && rm -f V2_03_difficult.zip 
    # V2
    fi

  fi  
}

# Check if gdown is installed
if [ python3 -c "import gdown" &> /dev/null ]; then
    echo "gdown is already installed."
else
    # Install gdown using pip
    echo "Installing gdown..."
    sudo pip3 install gdown && sudo pip install gdown
fi

if [ python3 -c "import rosbags" &> /dev/null ]; then
    echo "rosbags is already installed."
else
    # Install gdown using pip
    echo "Installing rosbags..."
    sudo pip3 install rosbags

    # Check if the installation was successful
    if [ $? -eq 0 ]; then
      echo "Rosbags installed successfully."
    else
      echo "Failed to install rosbags, Python >3.8 required. Exitting..."
      exit 1
    fi
fi


# define a registry to push the images to
ROOT_DIR="$PWD"
DATASET_DIR="$ROOT_DIR/datasets"
EUROC_DIR="$DATASET_DIR/EuRoC"
TUM_DIR="$DATASET_DIR/TUM"
ROS_VERSION="2"

while getopts e:t:r: flag
do
    case "${flag}" in
        e) EUROC_LIST=${OPTARG};;
        t) TUM_LIST=${OPTARG};;
        r) ROS_VERSION=${OPTARG};;
    esac
done


if [ ! -d $EUROC_DIR ]; then
  echo "Directory for EuRoC dataset not exists, creating it to path ${EUROC_DIR}..."
  mkdir -p $EUROC_DIR
fi

if [ ! -d $TUM_DIR ]; then
  echo "Directory for TUM dataset not exists, creating it to path ${TUM_DIR}..."
  mkdir -p $TUM_DIR
fi




if [ "${EUROC_LIST}" == "all" ]; 
then
  echo "Downloading all EuRoC datasets..."
  EUROC_LIST="MH_01,MH_02,MH_03,MH_04,MH_05,V1_01,V1_02,V1_03,V2_01,V2_02,V2_03"
fi


IFS=',' read -ra ADDR <<< "$EUROC_LIST"
for i in "${ADDR[@]}"; do
  download_euroc_dataset $i
done

if [ "$TUM_LIST" == "all" ]; then
  echo "Downloading all TUM datasets..."
  TUM_LIST="room1,room2,room3,room4,room5,room6"
fi

IFS=',' read -ra ADDR <<< "$TUM_LIST"
for i in "${ADDR[@]}"; do
  download_tum_dataset $i
done






