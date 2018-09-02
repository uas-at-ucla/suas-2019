fusermount -u /tmp/drone_code_dslr_mounted


TIME="$(date +%s)"; gphoto2 --auto-detect --set-config /main/settings/datetime="${TIME}" --set-config /main/imgsettings/imageformat=1 --force-overwrite --set-config imageformat=3  --set-config capturetarget=1 --set-config iso=0 --set-config eosremoterelease=5 --wait-event=100s --set-config eosremoterelease=4
