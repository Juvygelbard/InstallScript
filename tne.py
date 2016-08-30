from subprocess import Popen, PIPE, STDOUT
from time import sleep

# proc = Popen("konsole", stdin=PIPE)
# sleep(1)
# proc.communicate(input=b'echo hi')

p = Popen("source", stdout=PIPE, stdin=PIPE, stderr=STDOUT)
grep_stdout = p.communicate(input=b'one\ntwo\nthree\nfour\nfive\nsix\n')[0]
print(grep_stdout.decode())