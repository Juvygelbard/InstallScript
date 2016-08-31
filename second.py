from subprocess import call
from os.path import expanduser
from xml.etree import ElementTree
from sys import argv

# second version: build a new sh script for each run

# TODO: handle 'source'
# TODO: add 'cond' option
# TODO: add 'tree' command

INSTALL_XML = 'deps.xml'
LIST = False
ASK = False
NO_INST = False
NO_INST_LIST = []
ONLY_INST = False
ONLY_INS_LIST = []

# parse cmd line args
for i in range(1, len(argv)):
    if argv[i]=='-l' or argv[i]=='--list':
        LIST = True
    elif argv[i]=='-a' or argv[i]=='--ask':
        ASK = True
    elif argv[i]=='-d' or argv[i]=="--dont_install":
        NO_INST = True
        NO_INST_LIST = argv[i+1:len(argv)]
        break
    elif argv[i]=='-r' or argv[i]=="--roots":
        ONLY_INST = True
        ONLY_INS_LIST = argv[i+1:len(argv)]
        break

# class represents a single executable bash
class Bash:
    def __init__(self, dep, raw_bash, dir=None):
        self.dep = dep
        self.bash = [ln.strip() for ln in raw_bash.strip().splitlines()]
        self.dir = dir

    def execute(self, file):
        # execute bash line
        for ln in self.bash:
            print(ln)
            # dir and non dir cases
            if self.dir!=None:
                execode = call(ln, cwd=expanduser(self.dir), shell=True)
            else:
                execode = call(ln, shell=True)
            if execode:
                raise RuntimeError("Error while executing dependency '%s': '%s'" %(self.dep, ln))

# class represents a single dependency
class Dep:
    def __init__(self, name):
        self.name = name
        self.bashes = []
        self.deps = {}

    def add_dep(self, dep):
        self.deps.update({dep.name:dep})

    def get_dep(self, dep_name):
        if dep_name in self.deps:
            return self.deps[dep_name]

    def get_deps(self):
        return self.deps.values()

    def remove_dep(self, dep_name):
        if dep_name in self.deps:
            self.deps.pop(dep_name)

    def add_bash(self, raw_bash, dir=None):
        self.bashes.append(Bash(self.name, raw_bash, dir))

    def execute(self):
        print("[Executing Dependency: %s]" %self.name)
        # execute bashes
        for bash in self.bashes:
            bash.execute()

# class represents a full dependency tree
class DepTree:
    def __init__(self, filename):
        self.parse_xml(filename)

    # parses xml and returns a dependency tree
    def parse_xml(self, filename):
        et = ElementTree.parse(filename).getroot()

        # check root attribute
        if et.tag != 'install':
            raise RuntimeError("'%s' is not an installation xml file" %filename)

        # maps name->new dep
        self.deps = {}
        # maps yet-to-be-resolved-dep->dep asking for it
        self.dep_in_me = {}

        for raw_dep in et.findall('dep'):
            # build this dep
            dep_name = raw_dep.get('name')
            new_dep = Dep(dep_name)
            for bash in raw_dep.findall('run'):
                dir = bash.get('dir')
                raw_bash = bash.text
                new_dep.add_bash(raw_bash, dir)
            self.deps.update({dep_name: new_dep})

            # add my deps to be resolved later
            for prev_dep in raw_dep.findall('dep'):
                if prev_dep.text in self.dep_in_me:
                    self.dep_in_me[prev_dep.text].append(new_dep)
                else:
                    self.dep_in_me.update({prev_dep.text:[new_dep]})

        # resolve deps
        for (curr_dep_name, dep_list) in self.dep_in_me.items():
            if not curr_dep_name in self.deps:
                raise RuntimeError("Cannot resolve dependency: %s" %curr_dep_name)
            curr_dep = self.deps[curr_dep_name]
            for dep in dep_list:
                dep.add_dep(curr_dep)

        # find root
        root_name = et.get('root')
        if not root_name in self.deps:
            raise RuntimeError("Cannot resolve root: %s" % root_name)

        self.roots = [self.deps[root_name]]

    # remove a dependency branch from the tree
    def cut_branch(self, branch):
        if branch in self.dep_in_me:
            for dep in self.dep_in_me[branch]:
                dep.remove_dep(branch)

    def execute(self, ask=False):
        self.__executed__ = []
        for root in self.roots:
            self.__rec_execute__(root, ask)

    # helper recursive, to be used by execute()
    def __rec_execute__(self, dep, ask):
        for prev_dep in dep.get_deps():
            # don't execute if you already have
            if not prev_dep.name in self.__executed__:
                self.__rec_execute__(prev_dep, ask)

        ans = ''
        if not ask: ans = 'y'
        while ans!='y' and ans!='n':
            print("Should I install '%s'? [y/n]" %dep.name)
            ans = input()
        if ans=='y':
            dep.execute()
        self.__executed__.append(dep.name)

    def list(self):
        self.__list_count__ = 0
        self.__listed__ = []
        print("[Installation Order]")
        for root in self.roots:
            self.__rec_list__(root)

    # helper recursive, to be used by list()
    def __rec_list__(self, dep):
        for prev_dep in dep.get_deps():
            # make sure no dep is printed more then once
            if not prev_dep.name in self.__listed__:
                self.__rec_list__(prev_dep)
        self.__listed__.append(dep.name)
        self.__list_count__ += 1
        print("%3i   %s" %(self.__list_count__, dep.name))

dt = DepTree(INSTALL_XML)
if NO_INST:
    for dep in NO_INST_LIST:
        if dep==dt.roots[0].name:
            raise RuntimeError("Cannot cut root branch installation")
        dt.cut_branch(dep)

if ONLY_INST:
    dt.roots = []
    for root_name in ONLY_INS_LIST:
        if not root_name in dt.deps:
            raise RuntimeError("Cannot resolve root: %s" % root_name)
        dt.roots.append(dt.deps[root_name])

if LIST:
    dt.list()
else:
    dt.execute(ASK)