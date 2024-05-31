#!/bin/bash

# Define NoMachine version and download URL
NOMACHINE_VERSION="7.10.1"
NOMACHINE_URL="https://download.nomachine.com/download/${NOMACHINE_VERSION}/Linux/nomachine_${NOMACHINE_VERSION}_1_amd64.deb"

# Update the package list
echo "Updating package list..."
sudo apt update

# Install required dependencies
echo "Installing required dependencies..."
sudo apt install -y wget

# Download NoMachine package
echo "Downloading NoMachine package..."
wget ${NOMACHINE_URL} -O /tmp/nomachine.deb

# Install NoMachine
echo "Installing NoMachine..."
sudo dpkg -i /tmp/nomachine.deb

# Fix any missing dependencies
echo "Fixing any missing dependencies..."
sudo apt --fix-broken install -y

# Clean up
echo "Cleaning up..."
rm /tmp/nomachine.deb

# Verify installation
echo "Verifying installation..."
if dpkg -l | grep -q nomachine; then
    echo "NoMachine installation successful."
else
    echo "NoMachine installation failed."
fi
