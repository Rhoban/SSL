sudo ifconfig enp2s0 10.0.0.68 netmask 255.255.255.0
sudo route delete default
sudo route add default gw 10.0.0.254
route
