This Linux service enables MODAQ2 running on system boot. This is useful for running the system headless. You will need to edit the file paths to represent any differences on your computer.

Change m2 to your username

## PTP Setup
1. Determine what NIC your PTP master clock is connected to using the ifconfig command
2. In ptp4l.conf, change "enp2s0" to the name of you NIC (eth0, enp3s0, eth1, etc.)
3. In phc2sys.service change "enp2s0" to the name of your NIC used in step 2
4. Copy ptp4l.conf to /etc/linuxptp
5. Copy ptp4l.service to /lib/systemd/system
6. Copy phc2sys.service to /lib/systemd/system
7. Enable these service to start automatically on boot with "sudo systemctl enable ptp4l.service" and "sudo systemctl enable phc2sys.service"
8. Disable NTP on the device with "sudo timedatectl set-ntp false"
