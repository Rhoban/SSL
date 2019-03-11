
echo "Disabling multicast on loopback"
sudo route del -net 224.0.0.0 netmask 240.0.0.0 dev lo
sudo ifconfig lo -multicast