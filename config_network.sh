sudo ifconfig enp0s31f6 10.1.0.52 netmask 255.255.255.0
sudo route del default
sudo route add default gw 10.1.0.254
