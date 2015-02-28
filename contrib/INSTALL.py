#!/usr/bin/python
# Author: Anton Mitrokhin, MIPT 2014


class bcolors:
    HEADER = '\033[95m'
    PLAIN = '\033[37m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def offset(str_, p_offset):
    for i in xrange(p_offset):
        str_ = '...' + str_
    return str_

def hdr(str_, p_offset=0):
    return offset(bcolors.HEADER + str_ + bcolors.ENDC, p_offset)

def wht(str_, p_offset=0):
    return offset(bcolors.PLAIN + str_ + bcolors.ENDC, p_offset)

def okb(str_, p_offset=0):
    return offset(bcolors.OKBLUE + str_ + bcolors.ENDC, p_offset)

def okg(str_, p_offset=0):
    return offset(bcolors.OKGREEN + str_ + bcolors.ENDC, p_offset)

def wrn(str_, p_offset=0):
    return offset(bcolors.WARNING + str_ + bcolors.ENDC, p_offset)

def err(str_, p_offset=0):
    return offset(bcolors.FAIL + str_ + bcolors.ENDC, p_offset)

def bld(str_, p_offset=0):
    return offset(bcolors.BOLD + str_ + bcolors.ENDC, p_offset)


import sys, os, subprocess
import getpass

ROS_INSTALL_DIR = "/opt/ros/hydro"
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
LOCAL_BASH_FILE = ROOT_DIR + "/devel/setup.bash"
LAUNCHER_DIR = ROOT_DIR + "/contrib/launchers_gen/"
HOME_DIR = os.path.expanduser("~")
USERNAME = getpass.getuser()

try:
    ROS_HOSTNAME=os.environ["ROS_HOSTNAME"]
except:
    ROS_HOSTNAME=""

try:
    ROS_MASTER_URI=os.environ["ROS_MASTER_URI"]
except:
    ROS_MASTER_URI=""

try:
    GAZEBO_MODEL_PATH=os.environ["GAZEBO_MODEL_PATH"]
except:
    GAZEBO_MODEL_PATH=""

try:
    NUCOBOT_BASE=os.environ["NUCOBOT_BASE"]
except:
    NUCOBOT_BASE=""

try:
    NUCOBOT_3D_SENSOR=os.environ["NUCOBOT_3D_SENSOR"]
except:
    NUCOBOT_3D_SENSOR=""

try:
    NUCOBOT_SIMULATION=os.environ["NUCOBOT_SIMULATION"]
except:
    NUCOBOT_SIMULATION=""

print bld(wht("\nINSTALL SCRIPT FOR NUCOBOT PROJECT"))
print bld(wht("author: Anton Mitrokhin, 2014"))
print bld(wrn("Warning! "))+wht("This script assumes you had installed ROS Hydro to the ")+okb("'"+ROS_INSTALL_DIR+"'")
print wht("")
print wht("Workspace root directory: ")+okb(ROOT_DIR)
print wht("Launcher bash scripts directory: ")+okb(LAUNCHER_DIR)
print wht("")
print wht("Network setup:")
print hdr("ROS_HOSTNAME")+wht(": ")+okb(ROS_HOSTNAME)
print hdr("ROS_MASTER_URI")+wht(": ")+okb(ROS_MASTER_URI)
if (ROS_HOSTNAME == "" or ROS_MASTER_URI == ""):
    print err("Network setup is incomplete!")
    print wht("Set up ")+hdr("ROS_HOSTNAME")+wht(" and ")+hdr("ROS_MASTER_URI")+wht(" system variables")    
print wht("")
print wht("Other environment variables:")
print hdr("GAZEBO_MODEL_PATH")+wht(": ")+okb(GAZEBO_MODEL_PATH)
print hdr("NUCOBOT_BASE")+wht(": ")+okb(NUCOBOT_BASE)
print hdr("NUCOBOT_3D_SENSOR")+wht(": ")+okb(NUCOBOT_3D_SENSOR)
print hdr("NUCOBOT_SIMULATION")+wht(": ")+okb(NUCOBOT_SIMULATION)
print wht("")

def ensure_dir(f):
    if not os.path.exists(f):
        print okg("Created directory: ") + okb(f)
        os.makedirs(f)

def add_to_file(f, contents, string):
    if string not in contents:
        f.write(string + ' # Generated by NUCOBOT-INSTALL-PY\n')
        print okg("[ADDED]: ") + wht(string)
    else:
        print okg("[OK]: ") + wht(string)

def exec_command(cmd_):
    print bld(wht("Executing:"))
    print wht(cmd_, 1)
    print bld(wht(""))
    print bld(wht("Result:"))
    try:
        err_code = subprocess.call(["bash", "-c", cmd_])
        if (err_code != 0):
            print bld(err("Error: the return code is "+str(err_code)+". (Non-zero)"))
            print bld(wrn("If you want to return to this place restart the script."))
            return 1
    except:
        print bld(err("Something has went wrong! (" + str(sys.exc_info()) + ")")) 
        print bld(wrn("If you want to return to this place restart the script."))
        return 1
    return 0

def query(str_):
    valid = {"y": True, "n": False}
    sys.stdout.write(str_)
    while True:
        clr_str = ""
        choice = raw_input().lower()
        if choice in valid:
            return valid[choice]
        else:
            if (choice == ''): choice = ' '
            sys.stdout.write('\033[1A'+'\033['+str(len(str_))+'C')
            for l in choice: clr_str = clr_str + " "
            sys.stdout.write(clr_str)
            sys.stdout.write('\033['+str(len(clr_str))+'D')

def cleanup():
    print bld(wht("Cleaning up..."))
    output = []
    cnt = 0
    with open(HOME_DIR + "/.bashrc", 'r') as fbashrc:
        for line in fbashrc:
            cnt = cnt + 1
            if not 'Generated by NUCOBOT-INSTALL-PY' in line:
                cnt = cnt - 1
                output.append(line)

    with open(HOME_DIR + "/.bashrc", 'w') as fbashrc:
        fbashrc.writelines(output)
    
    if (cnt != 0):
        print wht("Removed ", 1)+err(str(cnt))+wht(" autogenerated lines in .bashrc")
    
    print wht("Removing shortcuts:", 1)
    
    for name in os.listdir(HOME_DIR + '/.local/share/applications/'):
        if '_ad_gen.desktop' in name:
            os.remove(HOME_DIR + '/.local/share/applications/' + name)
            print wht("removed ", 2) + okb(name)

def init_workspace():
    try:
        os.remove(ROOT_DIR + "/src/CMakeLists.txt")
    except:
        pass

    print "\nInitializing ROS workspace at " + ROOT_DIR + "/src";
    if not exec_command("source "+ ROS_INSTALL_DIR + "/setup.bash " +
                        "&& cd " + ROOT_DIR + "/src && catkin_init_workspace") == 0:
        print bld(err("Unable to execute catkin_init_workspace. Have you installed ROS?"))
        exit(1)

def add_shortcut(name, icon, command):
    # The generation is done analogicaly to alacarte utilite
    if not os.path.isfile(icon):
        print bld(err("No such icon: ")) + okb(icon)
        exit(1)

    file_path = HOME_DIR + '/.local/share/applications/' + name + '_ad_gen.desktop'
    format_str = '#!/usr/bin/env xdg-open\n\n'\
                 '[Desktop Entry]\n'\
                 'Version=1.0\n'\
                 'Type=Application\n'\
                 'Terminal=true\n'\
                 'Name=' + name + '\n'\
                 'Icon=' + icon + '\n'\
                 'Exec=' + command
    try:
        desktop_file = open(file_path, 'w+')
        desktop_file.write(format_str)
        os.chmod(file_path, 0775)
        print okg("Written to ")+okb(file_path)
    except:
        print bld(err("Error creating "))+okb(file_path)
        print sys.exc_info()
        exit(1)

def gen_launcher(bash_name, launcher_name, icon_name, command):
    format_str = '#!/bin/bash\n'\
    'source ' + ROS_INSTALL_DIR + '/setup.bash\n'\
    'source ' + LOCAL_BASH_FILE + '\n'\
    'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH\n'
    if len(str(ROS_HOSTNAME)) != 0:
        format_str +='export ROS_HOSTNAME=' + str(ROS_HOSTNAME) + '\n'
    if len(str(ROS_MASTER_URI)) != 0:
        format_str +='export ROS_MASTER_URI=' + str(ROS_MASTER_URI) + '\n'
    if len(str(GAZEBO_MODEL_PATH)) != 0:
        format_str +='export GAZEBO_MODEL_PATH=' + str(GAZEBO_MODEL_PATH) + '\n'
    format_str += '\n' + command + '\n\n'\
    'sleep 0.2'

    file_path = LAUNCHER_DIR + bash_name
    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print okg("Written to ")+okb(file_path)
    except:
        print bld(err("Error creating "))+okb(file_path)
        print bld(err("Check the permissions"))
        exit(1)

    add_shortcut(launcher_name, ROOT_DIR + '/contrib/icons/' + icon_name, file_path)

def setup_user_permissions():
    print hdr("You user (")+bld(okb(USERNAME))+hdr(") should be in a ")+err("'dialout'")+hdr(" group to be able to communicate with some sensor types.\n"+
              "Do you wish to be added? (Requires superuser privileges)") 
    if (query("y/n?")):
        print wht("Adding user " + USERNAME + " to 'dialout'...")
        exec_command("sudo adduser " + USERNAME + " dialout")
    else:
        print wht("Skipping...")


def install_project_deps(dep_list):
    pass




# Remove obsolete config paths
cleanup()

# Add myself to dialout and do other useful system stuff
setup_user_permissions()

# Install required packages
install_project_deps(["ros-hydro-ros-control"])

# Initialize catkin workspace
init_workspace() 

# Configure ~/.bashrc for ROS
with open(HOME_DIR + "/.bashrc", 'r+') as fbashrc:
    contents = fbashrc.read()
    add_to_file(fbashrc, contents, "source " + ROS_INSTALL_DIR + "/setup.bash")
    add_to_file(fbashrc, contents, "source " + LOCAL_BASH_FILE)
    add_to_file(fbashrc, contents, "export PATH=" + LAUNCHER_DIR + ":$PATH")
    add_to_file(fbashrc, contents, "export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH")

print wht("")

# Generate launchers
ensure_dir(LAUNCHER_DIR)

# gen_launcher('bashscriptname', 'launchername', 'iconname.png', 'roslaunch <package> <launchfile>.launch')
gen_launcher('ros_rebuild',                   'rosRebuild',         'rosRebuild.png',      'cd ' + ROOT_DIR + '\n\n'\
                                                                                           'catkin_make\n\n'\
                                                                                           'source ' + LOCAL_BASH_FILE + '\n'\
                                                                                           'echo "Press any key to continue..."\nread')

gen_launcher('ros_rebuild_eclipse',           'rosRebuild4Eclipse', 'rosRebuild4Eclipse.png', 'cd ' + ROOT_DIR + '\n'\
                                                                                           'catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug'\
                                                                                           ' -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8\n'\
                                                                                           'source ' + LOCAL_BASH_FILE + '\n'\
                                                                                           'echo "Press any key to continue..."\nread')

gen_launcher('ros_simulation',                'rosSimulation',      'rosSimulator.png',    'roslaunch nucobot_simulator nucobot_simulator.launch')


