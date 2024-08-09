#!/bin/bash

# Generate all the necessary files
echo "Generating IP routing files"

python3 gen_ip_route_files.py

# Move generated files
echo "Moving generating file"

sudo cp ./ip_route.service /etc/systemd/system/
sudo mv ./ip_route.sh /usr/local/sbin/
sudo mv ./fastdds_discovery_super_client.xml /etc/turtlebot4_discovery/
sudo mv ./setup.bash /etc/turtlebot4_discovery/

# Enable and start IP route service
echo "Enable/Restart service"

sudo systemctl enable ip_route.service
sudo systemctl restart ip_route.service

# End of script
echo "Sourcing ~/.bashrc file to apply changes"

source ~/.bashrc