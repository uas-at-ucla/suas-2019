fusermount -u /tmp/drone_code_dslr_mounted
rm -rf /tmp/drone_code_dslr_mounted

mkdir /tmp/drone_code_dslr_mounted

killall gphotofs
gphoto2 --auto-detect
/usr/bin/gphotofs /tmp/drone_code_dslr_mounted

rsync -avP --no-inc-recursive --progress /tmp/drone_code_dslr_mounted /home/pi/pictures
fusermount -u /tmp/drone_code_dslr_mounted
