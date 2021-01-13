#!/bin/bash

apt-get update -y
apt-get install -y git wget build-essential curl



export DEBIAN_FRONTEND=noninteractive
apt-get install -y --no-install-recommends tzdata
apt-get install -yq software-properties-common
add-apt-repository -y ppa:deadsnakes/ppa
apt-get install -y python3.7

apt-get update && apt-get install -y python3-pip