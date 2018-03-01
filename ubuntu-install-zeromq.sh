#!/usr/bin/env bash

# As in: http://stackoverflow.com/a/41289659/7331008

# Exiting on errors
set -e

# Set required version
VERSION="4.2.2"

# Asking for sudo password
sudo whoami

echo "Downloading ZeroMQ"
wget https://github.com/zeromq/libzmq/releases/download/v${VERSION}/zeromq-${VERSION}.tar.gz

echo "Unpacking"
tar xvzf zeromq-${VERSION}.tar.gz

echo "Installing dependencies"
sudo apt-get update && \
sudo apt-get install -y libtool pkg-config build-essential autoconf automake uuid-dev

echo "Changing directory"
cd zeromq-${VERSION}

echo "Configuring"
./configure

echo "Compiling"
make

echo "Installing"
sudo make install

echo "Installing ZeroMQ driver"
sudo ldconfig

echo "Checking installation"
ldconfig -p | grep zmq

