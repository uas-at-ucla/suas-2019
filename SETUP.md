# Setting Up UAS Development Environment
Please read through the entire documentation first before running anything on your machine.

## Contents
 * Operating System
   * Linux Setup
   * Windows Setup
   * MacOS Setup
 * Git Control
   * GitHub
   * SSH Setup
   * Cloning
   * Branching
     * Pulling
     * Pushing and Merge Conflicts
   * Pull Request Process
 * Compiling Programs

## Operating System

### Linux Setup
Linux is the premiere way we would like to develop all of our code. Ubuntu 16.04 is the recommended distribution due to its popularity, functional device drivers, and low-risk of bugs. You can install Linux in two ways:
1. Install a virtual machine.
2. If you have a working Windows machine, you can dual boot it to run Ubuntu natively on your own machine.

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
pip3 install -r /path/to/suas_2018/build/pip_requirements.txt
```
Executing Python scripts.
'''bash
python ./file_name.py
'''

## Virtual Machine
Install a [Virtual Machine](https://www.virtualbox.org/wiki/Downloads).

Get the ISO image.


## Git Control
1. Create a Git Account
2. Add SSH-key onto your local machine or VM
   https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
3.