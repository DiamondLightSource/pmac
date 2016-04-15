#!/bin/bash
# Make sure we exit on any error
set -e

# Enabling the NSLS-II EPICS debian package repositories
curl http://epics.nsls2.bnl.gov/debian/repo-key.pub | sudo apt-key add -
echo "deb http://epics.nsls2.bnl.gov/debian/ wheezy main contrib" | sudo tee -a /etc/apt/sources.list
echo "deb-src http://epics.nsls2.bnl.gov/debian/ wheezy main contrib" | sudo tee -a /etc/apt/sources.list

# Installing the latest 3rd party packages: EPICS, hdf5, tiff, etc.
sudo apt-get update -qq
sudo apt-get install epics-dev libhdf5-serial-dev libtiff-dev libxml2-dev libboost-test-dev

# Installing latest version of code coverage tool lcov (because the ubuntu package is very old)
wget http://downloads.sourceforge.net/ltp/lcov-1.11.tar.gz
tar -zxf lcov-1.11.tar.gz
sudo make -C lcov-1.11 install
gem install coveralls-lcov
