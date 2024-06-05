# Event-Driven UAV Swarms

Install Clover Raspberry Pi Image for all the drones.

## Modifications in Leader's RPi

1.  SSH into the leader's RPi
    ```bash
    ssh pi@clover1

    password: 22
    ```
2.  Configure the leader as an access point.

    Go to ```wpa_suupplicant.conf``` using the following command:
    ```bash
    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
    ```
    Update ```wpa_supplicant.conf``` file:
    ```
    ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
    update_config=1
    country=GB
    network={
        ssid="clover1"
        psk="cloverwifi"
        mode=2
        proto=RSN
        key_mgmt=WPA-PSK
        pairwise=CCMP
        auth_alg=OPEN
    }
    ```
    Go to ```dhcpcd.conf``` file:
    ```bash
    sudo nano /etc/dhcpcd.conf
    ```
    Add the following lines at the end of ```dhcpcd.conf``` file:
    ```
    interface wlan0
    static ip_address=192.168.11.2/24
    ```
3.  Configure the ROS Network.
    
    Update ```.bashrc``` file:
    ```bash
    sudo nano .bashrc
    ```
    Comment the following lines:
    ```
    #export ROS_HOSTNAME=`hostname`.local
    #export ROS_HOSTNAME=192.168.11.x
    ```
    Add the following lines at the end of the ```.bashrc``` file:
    ```
    source /opt/ros/noetic/setup.bash
    source /home/pi/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI=http://192.168.11.2:11311
    export ROS_IP=192.168.11.2
    ```


############
Go to rc.local
sudo nano /etc/rc.local

update rc.local:
#!/bin/bash -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Print the IP address
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi

source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.11.2:11311
export ROS_IP=192.168.11.2

su - pi -c " source /opt/ros/noetic/setup.bash; source /home/pi/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://192.168.11.2:11311; export ROS_IP=192.168.11.2; /opt/ros/noetic/bin/roslaunch uav_swarms formation_control_final.launch"

exit 0
#################

##############
update clover.service:
cd /etc/systemd/system/
sudo nano clover.service

[Unit]
Description=Clover ROS package
Requires=roscore.service

[Service]
User=pi
ExecStart=/bin/bash -c ". /home/pi/catkin_ws/devel/setup.sh; \
                      ROS_HOSTNAME=`hostname`.local exec stdbuf -o L roslaunch uav_swarms leader.launch --wait --screen --skip-log-check \
                      2> >(tee /tmp/clover.err)"

ExecStartPre=+rm /var/log/clover.log
StandardOutput=file:/var/log/clover.log
StandardError=file:/var/log/clover.log

[Install]
WantedBy=multi-user.target
#####################

make sure to update mav_sysid parameter of each drone to 1 using mission_planner/QGC
################

####################################
Now update follower parameters:

Changes in follower's setup:
###########
ssh to follower
ssh pi@clover2
password: 22
###########

#########
configure follower as client:

Go to wpa_suupplicant.conf:
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

update wpa_supplicant.conf:

ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB
network={
    ssid="clover1"
    psk="cloverwifi"
}

##########

##########
Go to dhcpcd.conf
sudo nano /etc/dhcpcd.conf

add these two lines in the end:
interface wlan0
static ip_address=192.168.11.3/24
###############

##################
update .bashrc
sudo nano .bashrc

comment these lines:

#export ROS_HOSTNAME=`hostname`.local
#export ROS_HOSTNAME=192.168.11.x

add these lines in the end:
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.11.2:11311
export ROS_IP=192.168.11.3
###################

############
Go to rc.local
sudo nano /etc/rc.local

update rc.local:
#!/bin/bash -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Print the IP address
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi

source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.11.2:11311
export ROS_IP=192.168.11.3

su - pi -c " source /opt/ros/noetic/setup.bash; source /home/pi/catkin_ws/devel/setup.bash; export ROS_MASTER_URI=http://192.168.11.2:11311; export ROS_IP=192.168.11.3; /opt/ros/noetic/bin/roslaunch uav_swarms follower.launch"

exit 0
#################

##############
update clover.service:
cd /etc/systemd/system/
sudo nano clover.service

[Unit]
Description=Clover ROS package
Requires=roscore.service

[Service]
User=pi
ExecStart=/bin/bash -c ". /home/pi/catkin_ws/devel/setup.sh; \
                      ROS_HOSTNAME=`hostname`.local exec stdbuf -o L roslaunch uav_swarms follower.launch --wait --screen --skip-log-check \
                      2> >(tee /tmp/clover.err)"

ExecStartPre=+rm /var/log/clover.log
StandardOutput=file:/var/log/clover.log
StandardError=file:/var/log/clover.log

[Install]
WantedBy=multi-user.target
#####################

make sure to update mav_sysid parameter of each drone to 1 using mission_planner/QGC
################