#!/bin/bash

# Define the URL for the BalenaEtcher .AppImage file
ETCHER_URL="https://github.com/balena-io/etcher/releases/download/v1.19.21/balenaEtcher-1.19.21-x64.AppImage"

# Define the location to save the downloaded file
DEST_DIR="$HOME/Downloads"
DEST_FILE="$DEST_DIR/balenaEtcher-1.19.21-x64.AppImage"

# Function to download Etcher
download_etcher() {
    echo "Downloading BalenaEtcher..."
    wget -O "$DEST_FILE" "$ETCHER_URL"
    if [ $? -ne 0 ]; then
        echo "Failed to download BalenaEtcher."
        exit 1
    fi
    echo "Downloaded BalenaEtcher successfully."
}

# Function to make the AppImage executable
make_executable() {
    echo "Making BalenaEtcher executable..."
    chmod +x "$DEST_FILE"
    if [ $? -ne 0 ]; then
        echo "Failed to make BalenaEtcher executable."
        exit 1
    fi
    echo "BalenaEtcher is now executable."
}

# Function to create a desktop entry
create_desktop_entry() {
    echo "Creating desktop entry for BalenaEtcher..."
    DESKTOP_ENTRY="[Desktop Entry]
Name=BalenaEtcher
Exec=$DEST_FILE
Icon=$DEST_FILE
Type=Application
Categories=Utility;"

    echo "$DESKTOP_ENTRY" > ~/.local/share/applications/balenaEtcher.desktop
    if [ $? -ne 0 ]; then
        echo "Failed to create desktop entry."
        exit 1
    fi
    echo "Desktop entry created successfully."
}

# Main script execution
download_etcher
make_executable
create_desktop_entry

echo "BalenaEtcher installation completed successfully."
