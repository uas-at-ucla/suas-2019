# Setting Up UAS Development Environment
Please read through the entire documentation first before running anything on your machine.

## Contents
 * Operating System
   * Windows Setup
   * Virtual Machine
 * Git Control
   * Initial Setup
   * Branching
     * Creating Your Own Branch
     * Pulling, Pushing, and Merge Conflicts
   * Pull Request Process
 * Compiling
 * Other Reads

## Operating System

### Windows Setup
Linux is the premier way we want to develop all of our code. Ubuntu 16.04 is the recommended distribution due to its popularity, functional device drivers, and low-risk of bugs. You have two options to install Linux:

1. Install a Virtual Machine. This is the easiest option.
2. [Dual boot](https://www.youtube.com/watch?v=qNeJvujdB-0) Windows and Ubuntu in order to 
   natively run Linux on your own machine. You will need to [disable
   secure boot](https://docs.microsoft.com/en-us/windows-hardware/manufacture/desktop/boot-to-uefi-mode-or-legacy-bios-mode)
   and [BitLocker](https://www.youtube.com/watch?v=RT-Acsx549c) before starting the video.

### Virtual Machine
If you are running on MacOS, you will need a Virtual Machine.

0. If you are using a Windows machine, be sure to [enable virtualization in your BIOS](http://bce.berkeley.edu/enabling-virtualization-in-your-pc-bios.html).
1. Install a [Virtual Machine](https://www.virtualbox.org/wiki/Downloads).
2. Download the [Virtual Box Appliance for UAS](https://drive.google.com/uc?export=download&confirm=Fn7m&id=0Bx9wIeK4Bxa_bTJ5UzhOWGdsUzQ).
3. Open Oracle VM VirtualBox.
4. On the top-left corner, press File > Import Appliance...
5. Locate the 'UAS at UCLA Virtualbox (Ubuntu).ova' file and press 'Next'. Then press 'Import'.
6. A new virtual machine should show up in VirtualBox, which will be the machine that you can develop code on. Click on the machine in the left-hand bar, and click Start. 

   Note: VirtualBox has a variety of escape keys to manage transitioning from the virtual machine to your host machine. This key should be displayed at the bottom right of the window. To escape from the virtual machine environment, you should be able to press this key, release it, and then move your mouse out of the environment or use your native machine’s shortcuts to change windows.
8. If you have a 1440p or 1080p screen, you may need to [scale the resolution](https://askubuntu.com/questions/823529/how-do-you-set-scaling-for-a-high-dpi-monitor-and-low-dpi-monitor-independently) so that everything does not look so tiny.
9. Default user credentials

   **user:** uas

   **pass:** password

We will update this virtual machine as more dependencies or a change in build environment are needed. Pay attention to version numbers as these will be used to track the types of virtual machines being posted.

## Git Control

### Initial Setup
1. Create a GitHub Account and log in.
2. [Email your GitHub username](http://uclauas.com/contact.php) in order to be
   added onto the SUAS repository. If you do not have access to the SUAS
   repository, the rest of the procedure below will not work.
3. Install the 'Git' package if your machine does not already have it.
   ```bash
   sudo apt-get install git
   ```
4. Add an [SSH-Key](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) onto your local machine or VM.
5. Clone the repository - assure you are cloning with an SSH key.
   ```bash
   git clone git@github.com:uas-at-ucla/suas_2018.git
   cd suas_2018
   ```
### Branching
Branching is a system we use so that we can constantly develop our code in
increments. After you successfully clone the repository, do not edit the files
directly. You need to create your own branch off of MASTER.

The MASTER branch is the final product we release to for our drone. Our branching
system will look like this:

............................ --------------- ivan

............................ --------------- comran

............................ --------------- yaacov

MASTER Branch ------------ vansh

............................ --------------- .

............................ --------------- .

............................ --------------- .

............................ --------------- and so on

Everyone on the team will have their own branch to work on - you can think of
the branch as your own sandbox. You will submit your code through a 'Pull
Request', in which the respective leadership committee will review the code for
approval and merging. The purpose of this is to make sure your code does not break anything severe on the master branch.

#### Creating Your Own Branch
1. Create the remote branch. In the [UAS Repository](https://github.com/uas-at-ucla/suas_2018), by the
   left-side section you should see a box that says 'Branch: master'. Click on it,
   and create the remote branch. Name the branch by your first name. The
   remote branch is the cloud where you store your newly edited files and
   allow everyone else to see your edits.
2. In the suas_2018 directory, run the following:
   ```bash
   git fetch
   git checkout -b FIRST_NAME origin/FIRST_NAME
   ```

#### Pulling, Pushing, and Merge Conflicts
1. After you make some edits, you can check the difference between your
   code and the GitHub repository.
   ```bash
   git status
   git diff
   ```
2. It is good practice to develop your code incrementally and back-up your edits
   onto GitHub - it is very common for people to accidentally lose all their
   data.
   ```bash
   git add FILE_NAME
   git commit -m "Enter commit message"
   git push
   ```
3. If you work on two separate machines or want to obtain the latest MASTER
   updates, you can simply pull all the content from the GitHub repository.
   ```bash
   git checkout master
   git pull
   git checkout FIRST_NAME
   git merge master
   ```
4. If a merge conflict exists, a 'MERGE CONFLICT' message will appear immediately after the merge. Simply edit the files
   and fix them. If you are unsure what to delete or fix, ask others for help.

### Pull Request Process
Pull requests are what you submit for people to review your code. Reviewers will
see the latest file edits you made that do not exist in the MASTER branch.

1. Go to the [UAS Repository](https://github.com/uas-at-ucla/suas_2018).
2. Near the top section, you should see a series of tabs: Code, Issues, Pull
   Requests, Projects, Wiki, Insights. Click on the 'Pull Requests' tab.
3. On the right-side, click on 'New Pull Request'.
4. Choose your branch. Check the diffs to make sure the branch contains the
   changes you want. Do not push any more changes until the PR has been approved.
5. After the PR has been approved, pull from MASTER and compile.
   ```bash
   git checkout master
   git pull
   ```
6. DO NOT delete your branch after a successful merge. The reason being is for convenience. It is easier for you to constantly merge the master branch into your own branch. See step 3.

And that's all the commonly used git controls. There are other useful commands you can google if you ever run into special situations. If you are new or not well-familiar with git, you will grow use to it after consecutive use and it will become almost second nature to you.

## Compiling
Install all of the packages that you will need.
```bash
sudo apt-get update
sudo apt-get install git python3.5 python3-pip python3-dev build-essential
```

Install Python dependencies.
```bash
pip3 install -r /path/to/suas_2018/build/pip_requirements.txt
```

Execute Python scripts.
```bash
python ./file_name.py
```

## Other Reads
Each directory - control, vision, ground - contains their own README.md
file. Depending on what sub-team you are in, those README.md files will list out
greater detail on the sub-team's code functionalities and compilation.
