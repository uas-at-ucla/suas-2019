# Setting Up UAS Development Environment
Please read through the entire documentation first before running anything on your machine.

## Contents
 * Operating System
   * Windows Setup
   * MacOS Setup
 * Git Control
   * Initial Setup
   * Branching
     * Creating Your Own Branch
     * Pulling, Pushing, and Merge Conflicts
   * Pull Request Process
 * Compiling
 * Other Reads

## Operating System
<<<<<<< HEAD
test
### Linux Setup
Linux is the premiere way we would like to develop all of our code. Ubuntu 16.04 is the recommended distribution due to its popularity, functional device drivers, and low-risk of bugs. You can install Linux in two ways:
1. Install a virtual machine.
2. If you have a working Windows machine, you can dual boot it to run Ubuntu natively on your own machine.
=======

### Windows Setup
Linux is the premiere way want to develop all of our code. Ubuntu 16.04 is the recommended distribution due to its popularity, functional device drivers, and low-risk of bugs. You can install Linux in three ways:

1. Install a Virtual Machine. See MacOS Setup. If you are unsure whether you
   want to pursue a career in Computer Science, then it's highly recommended to
   just stick with a virtual machine client and to ignore the other options.
2. If you have a working Windows machine, you can [dual boot](https://www.youtube.com/watch?v=qNeJvujdB-0) it to run Ubuntu
   natively on your own machine. If you had never setup a Linux installation
   before, it's recommended to watch the whole video first before you
   actually start tinkering your device. Moreover, you will need to [disable
   secure
   boot](https://docs.microsoft.com/en-us/windows-hardware/manufacture/desktop/boot-to-uefi-mode-or-legacy-bios-mode)
   and [BitLocker](https://www.youtube.com/watch?v=RT-Acsx549c). It is strongly
   encouraged you to message on Slack if you have any confusions or unsure of
   anything. Make sure to back-up your sensitive documents and files - if your
   operating system gets corrupted, we can help reinstall a fresh Windows OS
   onto your machine again.
3. Using the Ubuntu Bash.
   I am not entirely sure if this will work. I'll do further testing later whether
   it can imitate the same libraries as a Linux machine.

### MacOS Setup
Install a [Virtual Machine](https://www.virtualbox.org/wiki/Downloads).

Get the ISO image from Comran.

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
directly. You need to create your own branch off of <MASTER>.

The <MASTER> branch is the final product we release to for our drone. Our branching
system will look like this:

............................ --------------- ivan

............................ --------------- comran

............................ --------------- yaacov

MASTER Branch --------------- vansh

............................ --------------- .

............................ --------------- .

............................ --------------- .

............................ --------------- and so on
>>>>>>> 816802b0558a91581f80c90961820dc493626a8a

Everyone on the team will have their own branch to work on - you can think of
the branch as your own sandbox. You will submit your code through a 'Pull
Request', in which the respective leadership committee will review the code for
approval and merging. The purpose of this is to make sure your code does not break anything severe on the master branch.

#### Creating Your Own Branch
1. Create the remote branch. In the [UAS Repository](https://github.com/uas-at-ucla/suas_2018), by the
   left-side section you should see a box that says 'Branch: master'. Click on it,
   and create the remote branch. You should name it by your first name. This
   remote branch is the cloud where you store your newly edited files on and
   what everyone else can see.
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
   on GitHub - it is very common for people to accidentally lose all their
   data. Save your local changes on GitHub.
   ```bash
   git add FILE_NAME
   git commit -m "Enter commit message"
   git push
   ```
3. If you work on two separate machines or want to obtain the latest <MASTER>
   updates, you can simply pull all the content from the GitHub repository.
   ```bash
   git checkout master
   git pull
   git checkout FIRST_NAME
   git merge master
   ```
4. If a merge conflict exists, the log should alert you. Simply edit the files
   and fix them. If you are unsure what to delete or fix, simply ask for help
   from others.

### Pull Request Process
Pull requests are what you submit for people to review your code. Reviewers will
see the latest file edits you made that do not exist in the <MASTER> branch.

1. Go to the [UAS Repository](https://github.com/uas-at-ucla/suas_2018).
2. Near the top section, you should see a series of tabs: Code, Issues, Pull
   Requests, Projects, Wiki, Insights. Click on the 'Pull Requests' tab.
3. On the right-side, click on 'New Pull Request'.
4. Choose your branch. Check the diffs to make sure the branch contains the
   changes you want. Do not push any more changes until the PR has been approved.
5. After the PR has been approved, pull from <MASTER> and compile.
   ```bash
   git checkout master
   git pull
   ```
And that's all the commonly used git controls. There are other useful commands you can google if you ever run into special situations.

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