
function download_dataset() {
  if [ -e "$EUROC_DIR/$1" ]; then
    echo "The path $EUROC_DIR/$1 already exists, skipping..."
  else
  
    if [ "$1" == "MH_01" ]; then
      cd $EUROC_DIR && gdown 1cbVXoOzsqOEkwCGAVw7QwP-npp2S_W0G
      cd $EUROC_DIR && mkdir MH_01 && unzip -j MH_01_easy.zip -d MH_01 && rm -f MH_01_easy.zip 
    elif [ "$1" == "MH_02" ]; then
      cd $EUROC_DIR && gdown 18-H4MJHabvuaVdfcreShOtAKf88kusqB
      cd $EUROC_DIR && mkdir MH_02 && unzip -j MH_03_easy.zip -d MH_02 && rm -f MH_02_easy.zip 
    elif [ "$1" == "MH_03" ]; then
      cd $EUROC_DIR && gdown 1cpueLI-XDHc_EpauNrgaw7K0-Y3Mv-r0
      cd $EUROC_DIR && mkdir MH_03 && unzip -j MH_03_medium.zip -d MH_03 && rm -f MH_03_medium.zip 
    elif [ "$1" == "MH_04" ]; then
      cd $EUROC_DIR && gdown 15dKMBC1pRX0ZyUpxKmH9u0Jqxex43yFO
      cd $EUROC_DIR && mkdir MH_04 && unzip -j MH_04_difficult.zip -d MH_04 && rm -f MH_04_difficult.zip 
    elif [ "$1" == "MH_05" ]; then
      cd $EUROC_DIR && gdown 1dLtZ_Mapo6CGUMdXskbnFGaf2maNKWQL
      cd $EUROC_DIR && mkdir MH_05 && unzip -j MH_05_difficult.zip -d MH_05 && rm -f MH_05_difficult.zip 
    fi

  fi  
}

# Check if gdown is installed
if [ python3 -c "import gdown" &> /dev/null ]; then
    echo "gdown is already installed."
else
    # Install gdown using pip
    echo "Installing gdown..."
    pip3 install gdown
fi


# define a registry to push the images to
ROOT_DIR="$PWD"
DATASET_DIR="$ROOT_DIR/datasets"
EUROC_DIR="$DATASET_DIR/EuRoC"

while getopts a:e: flag
do
    case "${flag}" in
        a) ALL_DATASETS=${OPTARG};;
        e) EUROC_LIST=${OPTARG};;    
    esac
done


if [ -n "${ALL_DATASETS}" ]; 
then
  echo "Downloading EuRoC dataset..."
else
  IFS=',' read -ra ADDR <<< "$EUROC_LIST"
  for i in "${ADDR[@]}"; do
    # process "$i"
    download_dataset $i
  done
fi





