#!/bin/bash
NC='\033[0m'       # Text Reset
Green='\033[0;32m'        # Green

PYTHON38_EXISTS=$(python3 -c 'import sys; print(sys.version_info[:])')
if [[ "${PYTHON38_EXISTS}" == *"(3, 8,"* ]]; then
	echo -e "${Green}Success${NC}: python3.8 found"
else
	echo "python3.8 not found! This virtual environment only works with python3.8, please install it first."
	exit
fi

# Check if virtual environment is installed
RESULT=$(dpkg -s python3-venv)
if [[ "${RESULT}" == *"install ok installed"* ]]; then
	echo -e "${Green}Success${NC}: python3-venv found"
else
	read -p "python3-venv is not installed, do you want to install it? [y/n] " -n 1 -r
	echo    # (optional) move to a new line
	if [[ $REPLY =~ ^[Yy]$ ]]
	then
		sudo apt-get install python3-venv
	fi
fi

# Source the virtual environment and install packages
echo "Setting up virtual environment..."
python3.8 -m venv venv
source venv/bin/activate
pip3 install numpy==1.18.3 scipy==1.4.1 casadi==3.5.1 requests==2.26.0
deactivate
echo -e "${Green}Done!${NC}"