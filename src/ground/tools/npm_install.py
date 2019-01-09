import os
import sys
import subprocess

def npm_install():
    if os.path.isfile("package.json"):
        check_deps = "./node_modules/.bin/check-dependencies"
        if (not os.path.isfile(check_deps)) or (subprocess.call(check_deps) != 0):
            subprocess.call(["npm", "install"])
    else:
        print("Error: package.json not present")

if __name__ == '__main__':
    npm_install()