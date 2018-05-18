fusermount -u camera_mounted
gphoto2 --set-config capturetarget=1 --set-config drivemode=1 --set-config eosremoterelease=2 --list-config --wait-event=3s --set-config eosremoterelease=0
mkdir camera_mounted
mkdir downloaded
gphotofs camera_mounted

rsync -avP --progress camera_mounted downloaded
