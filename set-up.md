# Setting Up UAS Development Environment
Please read through the entire documentation first before running anything on your machine.

## Linux Set-up
Install all of the packages that you will need.
```bash
sudo apt-get update
sudo apt-get install git python3.5 python3-pip python3-dev build-essential 
```

Download this repository.
```bash
cd ~/Documents
git clone https://github.com/uas-at-ucla/suas_2018.git
cd suas_2018
```

Install Python dependencies.
```bash
pip3 install -r build/install/requirements.txt
```


## Virtual Machine
https://www.virtualbox.org/wiki/Downloads


## Setting Up Git Control
1. Create a Git Account
2. Add SSH-key onto your local machine or VM
   https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
3.