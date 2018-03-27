import sys
import os
import signal

# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

sys.path.insert(0, '../../../lib')
import process_manager

processes = None


def signal_received(signal, frame):
    processes.killall()
    sys.exit(0)


if __name__ == "__main__":
    processes = process_manager.ProcessManager()
    signal.signal(signal.SIGINT, signal_received)

    build = False

    git_status = processes.return_output("git status")
    if "src/" in git_status or "public/" in git_status \
       or "package.json" in git_status:
        print "React files have been modified. Building..."
        build = True

    latest_commit = processes.return_output("git show")
    latest_commit_id = latest_commit.split('\n')[0].split(' ')[1]
    if os.path.isfile("build_commit.txt"):
        with open("build_commit.txt") as file:
            build_commit_id = file.read()
            if latest_commit_id != build_commit_id:
                print "Detected new commit. Building React App..."
                build = True
    else:
        print "Building React App..."
        build = True

    if build:
        build_exit_code = processes.run_command("npm run build")
        if build_exit_code == 0:
            with open("build_commit.txt", 'w') as file:
                file.write(latest_commit_id)
    else:
        print "React build is up to date!"
