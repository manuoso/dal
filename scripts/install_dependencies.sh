#!/bin/sh

ubuntu_version=$(lsb_release -r)
ubuntu_version=$(cut -f2 <<< "$ubuntu_version")

mkdir ~/.dal/ -p
cd ~/.dal/
mkdir thirdparty -p
cd thirdparty

install_git_repo () {
	read -r -p "Do you want to install $1 [y/N] " response
	if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
	then
		if [ -d "./$1" ] 
		then
			echo "Library $1 already installed" 
		else
			git clone $2
			cd $1
			if [ -z "$3" ]   # Is parameter #1 zero length?
			then
				git checkout "$3"
			fi
			
			mkdir build ; cd build
			cmake ..
			make -j$(nproc)
			sudo make install 
			cd ../..
		fi
	fi
}

###################################################################
###########		       INSTALL APT LIBRARIES  		    ###########
###################################################################

sudo apt-get install libeigen3-dev

###################################################################
###########		    INSTALL CMAKE LAST RELEASE		    ###########
###################################################################

read -r -p "Do you want to install latest version of CMake [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
	git clone -b "release" "https://github.com/Kitware/CMake"
	cd CMake
	./bootstrap ; make ; sudo make install
	cd ..
fi

###################################################################
###########				INSTALL Onboard-SDK			    ###########
###################################################################
install_git_repo "Onboard-SDK" "https://github.com/dji-sdk/Onboard-SDK" "3.9"

###################################################################
###########				INSTALL rapidjson			    ###########
###################################################################
install_git_repo "rapidjson" "https://github.com/Tencent/rapidjson"
