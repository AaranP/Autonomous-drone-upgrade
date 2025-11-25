#Fixes the issue with building x86 images on ARM host
docker run --privileged --rm tonistiigi/binfmt --install all
#optional check of installed emulators
ls /proc/sys/fs/binfmt_misc
#Actual build command to build the docker image for x64 
docker build --platform linux/amd64 -t fast_drone_noetic .


#It would take like 10 -15 minutes to build