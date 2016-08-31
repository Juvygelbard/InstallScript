from subprocess import Popen, PIPE, call
from os.path import expanduser
from os import environ

# p = Popen(expanduser("sudo ~/catkin_ws/devel/setup.bash"))
env_1={}
print(len(env_1))
p = Popen("./a.sh", shell=True, env=env_1, stdout=PIPE)
raw_vars = p.communicate()[0].decode()
vars = raw_vars.splitlines()
print(len(vars))
print(raw_vars)