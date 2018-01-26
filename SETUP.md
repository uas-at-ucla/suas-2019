# Setting Up UAS Development Environment

## Contents
 * Git Control
   * Initial Setup
   * Branching
     * Creating Your Own Branch
     * Pulling, Pushing, and Merge Conflicts
   * Pull Request Process
 * Operating System
   * Windows
   * Linux and MacOS

## Git Control
### Initial Setup
1. Create a GitHub Account and log in.
2. Message in Slack Comran or in the #random channel your GitHub username in order to be
   added onto the SUAS repository. If you do not have access to the SUAS
   repository, the rest of the procedure below will not work.
3. Install the 'Git' package if your machine does not already have it.
   ```bash
   sudo apt-get install git
   ```
4. Add an [SSH-Key](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) onto your local machine or VM.
5. Clone the repository - assure you are cloning with an SSH key.
   ```bash
   git clone --recursive git@github.com:uas-at-ucla/suas_2018.git
   cd suas_2018
   ```
### Branching
Branching is a system we use so that we can constantly develop our code in
increments. After you successfully clone the repository, do not edit the files
directly. You need to create your own branch off of MASTER.

The MASTER branch is the final product we release to for our drone.
Everyone on the team will have their own branch to work on - you can think of
the branch as your own sandbox. You will submit your code through a 'Pull
Request', in which the respective leadership committee will review the code for
approval and merging. The purpose of this is to make sure your code does not
break anything severe on the master branch.

#### Creating Your Own Branch
1. Create the remote branch. In the [UAS Repository](https://github.com/uas-at-ucla/suas_2018), by the
   left-side section you should see a box that says 'Branch: master'. Click on it,
   and create the remote branch. Name the branch by the corresponding GitHub issue. The
   remote branch is the cloud where you store your newly edited files and
   allow everyone else to see your edits.
2. In the suas_2018 directory, run the following:
   ```bash
   git fetch
   git checkout -b GITHUB-ISSUE origin/GITHUB-ISSUE
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
   git checkout GITHUB-ISSUE
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
6. Do not delete your branch after a successful merge. The reason being is for convenience. It is easier for you to constantly merge the master branch into your own branch. See step 3.

And that's all the commonly used git controls. There are other useful commands you can google if you ever run into special situations. If you are new or not well-familiar with git, you will grow use to it after consecutive use and it will become almost second nature to you.

## Operating System

### Windows
1. Download [Visual Studio Code](https://code.visualstudio.com/docs/setup/windows).

2. Enable **Developer Mode**. Press the `Windows` key, search "For developers settings", and enable **Developer mode**.

3. Enable Windows Subsystem for Linux.
   To enable the WSL, open a PowerShell instance as the Administrator user. To do this, press the `Windows` key, start typing "PowerShell", and right click the main result and select "Run as administrator".

   With the shell open, run the below command and then reboot your machine.
   ```bash
   Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
   ```

4. To install bash, open a Command prompt by opening the start menu and typing in "cmd" and opening the first result. With it open, type 
   in "bash" and hit enter to install bash. This will handle downloading it, and creating the local Unix user.

5. Open up Visual Studio Code.

6. Change the default terminal to **WSL Bash**. Press `F1` in VS code, and type **Terminal: Select Default Shell**.

7. To open and close the integrated terminal in VS code, press `Ctl` + `&grave` simultaneously.

8. Follow the Linux and MacOS section below to proceed.

### Linux and MacOS
The `install.sh` file in the `build` directory abstracts the entire
installation process.
```bash
cd /path/to/build/
chmod +x install.sh
./install.sh
```

After successful installation, the ground control software can be run using:
```bash
python control/run.py
```

To start only the interop server, run ```python ground/run_interop.py```. Check it out at http://localhost:8000.

Use username 'testadmin' and password 'testpass'.
