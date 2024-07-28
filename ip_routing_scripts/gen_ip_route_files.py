#!/bin/python

import os

def main():
  ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID'))
  discovery_ip  = f"192.168.0.2{ros_domain_id:02d}"
  
  # IP Route
  with open("ip_route.sh.template", "r") as file_in:
    with open("ip_route.sh", "w+") as file_out:
      for line in file_in:
        ln = line.replace("_DISCOVERY_IP_", discovery_ip)
        file_out.write(ln)
        
  # Setup 
  with open("setup.bash.template", "r") as file_in:
    with open("setup.bash", "w+") as file_out:
      for line in file_in:
        ln = line.replace("_DISCOVERY_IP_", discovery_ip)
        ln = ln.replace("_ROS_DOMAIN_ID_", str(ros_domain_id))
        file_out.write(ln)

  # Discovery Super Client
  with open("fastdds_discovery_super_client.xml.template", "r") as file_in:
    with open("fastdds_discovery_super_client.xml", "w+") as file_out:
      for line in file_in:
        ln = line.replace("_DISCOVERY_IP_", discovery_ip)
        file_out.write(ln)


if __name__ == "__main__":
  main()