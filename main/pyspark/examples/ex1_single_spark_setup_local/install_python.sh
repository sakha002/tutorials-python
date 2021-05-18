#!/bin/bash

apt-get update -y
apt-get install -y git wget build-essential curl



export DEBIAN_FRONTEND=noninteractive
apt-get install -y --no-install-recommends tzdata
apt-get install -yq software-properties-common
add-apt-repository -y ppa:deadsnakes
apt-get install -y python3

# apt-cache show python3
# apt-get install python3=3.5.1*

# apt-get install libssl-dev openssl
# wget https://www.python.org/ftp/python/3.4.2/Python-3.4.2.tgz
# tar xzvf Python-3.4.2.tgz

# apt-get build-dep python

# cd Python-3.4.2
# ./configure 
# make
# make install

# ./configure --with-zlib
# make
# make altinstall

# add-apt-repository ppa:deadsnakes/ppa
# apt-get update
# apt-get install python3.4

apt-get update && apt-get install -y python3-pip