#!/bin/bash
set -euxo pipefail
sudo apt update && sudo apt upgrade -y
# Install X11 server 
sudo apt install -y xorg xinit

# Install i3
/usr/lib/apt/apt-helper download-file https://debian.sur5r.net/i3/pool/main/s/sur5r-keyring/sur5r-keyring_2024.03.04_all.deb ~/keyring.deb SHA256:f9bb4340b5ce0ded29b7e014ee9ce788006e9bbfe31e96c09b2118ab91fca734
sudo apt install ~/keyring.deb
echo "deb http://debian.sur5r.net/i3/ $(grep '^DISTRIB_CODENAME=' /etc/lsb-release | cut -f2 -d=) universe" | sudo tee /etc/apt/sources.list.d/sur5r-i3.list
sudo apt update
sudo apt install -y i3
sudo apt install -y terminator

# setup i3 and display on boot
sudo printf "#!/bin/bash\nexec i3" > ~/.xinitrc
sudo printf "\nif [[ -z \$DISPLAY ]] && [[ \$(tty) = /dev/tty1 ]]; then \n\tstartx\nfi\n" >> ~/.bashrc
sudo mkdir -p ~/.config/i3
sudo cp config ~/.config/i3/

# Install network manager
sudo apt install -y network-manager

# reboot
sudo reboot
