fusermount -u /tmp/suas_2018_dslr_mounted
rm -rf /tmp/suas_2018_dslr_mounted

mkdir /tmp/suas_2018_dslr_mounted
mkdir downloaded

killall gphotofs
/usr/bin/gphotofs /tmp/suas_2018_dslr_mounted

rsync -avP --progress /tmp/suas_2018_dslr_mounted downloaded
fusermount -u /tmp/suas_2018_dslr_mounted
