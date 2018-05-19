fusermount -u /tmp/suas_2018_dslr_mounted
rm -rf /tmp/suas_2018_dslr_mounted

mkdir /tmp/suas_2018_dslr_mounted
mkdir downloaded

killall gphotofs
gphotofs --port 8081 /tmp/suas_2018_dslr_mounted

rsync -avP --progress /tmp/suas_2018_dslr_mounted downloaded
