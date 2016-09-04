# Instool
## Introduction
This tool was created in-order to provide a simple and modular installation tool for the bgu komodo project.
It uses an XML which represents an installation dependency tree.
When executed, instool.py compiles the XML to a Linux bash script and runs it.
Note the instool.py should be executed using python3.

## Komodo Installation

### Prerequisites:
1. Ubunto 14.04
2. Git
3. Make sure Ubuntu repositories are configured to allow "restricted," "universe," and "multiverse".
Instructions can be found here: https://help.ubuntu.com/community/Repositories/Ubuntu


### Instructions:
1. Get the code:

```
mkdir ~/inst
cd ~/inst
git init
git clone https://github.com/Juvygelbard/InstallScript
```

2. Run installation:
```
python3 instool.py
```

3. Different components of the installation require your approval, so don't just leave it and go!

## Added Control

You may run the instool.py with different flags to modify the installation or to aquire information.

- -a / --ask := Ask user before installing each dependency (usefull to keep track of the installation).
- -l / --list := List dependencies to be installed by order of installation.
- -t / --tree := Display dependency tree.
- -d / --dep [dep1] [dep2] ... := Install/display only given dependencies (and their dependencies).
- -p / --dep-no-prev [dep1] [dep2] ... := Install/display only given dependencies (without their dependencies).
- -r / --remove-dep [dep1] [dep2] ... := Install/display all dependencies besides the given depenndencies (and theire deoendencies).
- -c / --compile := Compile the inst.sh file without running it.

### Current Issues
- After installing ROS Indigo, the terminal window should be restarted in order for it to recognize ROS and catkin tools. If the installation runs into an error at some point this is probably the reason, and one should just restart the terminal window and continue with the installation.
- The gazebo simulation is currently running but the speech module dosn't seems to recognize my commands.
