### Compiling and mouting the image onto the pi
- The image is compiled using the DOCKERFILE and ran using buildocker_window.sh on a machine with docker engine installed
- After compiling there will be a .tar file, copy that into a usb and insert into the PI
- On the pi the USB needs to be mounted so that it can be read 
```
sudo mkdir -p /media/usb_drive
sudo mount /dev/sda1 /media/usb_drive
```
- Thereafter copy the .tar file into the destination which in our case is /home/ledrone/Autonomous-drone-upgrade/
```
sudo cp /media/usb_drive/fastdrone_image.tar /home/ledrone/Autonomous-drone-upgrade/
```
- Then you need to load the image so that it can be read
```
docker load -i /home/ledrone/Autonomous-drone-upgrade/fastdrone_image.tar
```
These steps needs to be repeated if the source cpp code is modified and needs to be recompiled