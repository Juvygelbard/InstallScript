from subprocess import call
from os.path import dirname, abspath, expanduser
from os import chmod, stat, remove
from stat import S_IEXEC
from xml.etree import ElementTree
from sys import argv

# final [compiler] version, including cond support and help

INSTALL_XML = 'deps.xml'
COMPILED_BASH = 'inst.sh'
LIST = False
ASK = False
COMPILE = False
NO_INST = False
ONLY_INST = False
NO_PREV = False
TREE = False

MY_DIR = dirname(abspath(__file__))
PREF_STR = "	"

# parse cmd line args
for i in range(1, len(argv)):
    if argv[i]=='-l' or argv[i]=='--list':
        LIST = True
    elif argv[i] == '-t' or argv[i] == '--tree':
        TREE = True
    elif argv[i]=='-a' or argv[i]=='--ask':
        ASK = True
    elif argv[i]=='-c' or argv[i]=='--compile':
        COMPILE = True
    elif argv[i]=='-r' or argv[i]=="--remove-dep":
        NO_INST = True
        NO_INST_LIST = argv[i+1:len(argv)]
        break
    elif argv[i]=='-d' or argv[i]=="--dep":
        ONLY_INST = True
        ONLY_INST_LIST = argv[i+1:len(argv)]
        break
    elif argv[i] == '-p' or argv[i] == "--dep-no-prev":
        NO_PREV = True
        NO_PREV_LIST = argv[i + 1:len(argv)]
        break

class Cond:
    def __init__(self, name, raw_cond, dir=None, complement=False):
        self.name = name
        self.cond = [ln.strip() for ln in raw_cond.strip().splitlines() if ln.strip()[0]!="#"]
        self.complement = complement
        # if no dir val, use absolute path
        if dir is None:
            self.dir = MY_DIR
        else:
            self.dir = expanduser(dir)

    def to_file(self, prev_dir, file, pref):
        if prev_dir != self.dir:
            file.write("%scd %s\n" %(PREF_STR*pref, self.dir))
        for i, ln in enumerate(self.cond):
            file.write("%s%s\n"
                       "%scond%i=$?\n"
                       "%s" %(PREF_STR*pref, ln, PREF_STR*pref, i, PREF_STR*pref))
        file.write("if")
        if self.complement:
            file.write(" !(")
        for i in range(len(self.cond)):
            file.write(" [ $cond%i == 0 ]" %i)
            if i < len(self.cond)-1:
                file.write(" &&")
        if self.complement:
            file.write(")")
        file.write("; then\n")
        return self.dir

# class represents a single executable bash
class Bash:
    def __init__(self, dep, raw_bash, cond=None, dir=None):
        self.dep = dep
        self.bash = [ln.strip() for ln in raw_bash.strip().splitlines()]
        # if no dir val, use absolute path
        if dir is None:
            self.dir = MY_DIR
        else:
            self.dir = expanduser(dir)
        self.cond = cond

    def to_file(self, file, prev_dir, pref):
        if not self.cond is None:
            prev_dir = self.cond.to_file(prev_dir, file, pref)
            pref += 1
        # cd dir if needed
        if prev_dir != self.dir:
            file.write("%scd %s\n" % (PREF_STR*pref, self.dir))
        # write bash line + check for error
        for ln in self.bash:
            file.write("%s%s\n"
                       "%sres=$?; if [ $res != 0 ]; then exit $res; fi\n" %(PREF_STR*pref, ln, PREF_STR*pref))
        if not self.cond is None:
            pref -= 1
            file.write("%sfi\n" %(PREF_STR*pref))
        return self.dir

# class represents a single dependency
class Dep:
    def __init__(self, name):
        self.name = name
        self.bashes = []
        self.deps = {}
        self.conds = {}

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

    def clear_deps(self):
        self.deps.clear()

    def add_bash(self, raw_bash, cond=None, dir=None):
        self.bashes.append(Bash(self.name, raw_bash, cond, dir))

    def add_cond(self, name, raw_cond, dir=None, complement=False):
        self.conds.update({name : Cond(name, raw_cond, dir, complement)})

    def to_file(self, file, prev_dir, pref):
        file.write("%secho \"[Installing Dependency: %s]\"\n" %(PREF_STR*pref, self.name))
        # write bashes
        for bash in self.bashes:
            prev_dir = bash.to_file(file, prev_dir, pref)
        return prev_dir

# class represents a full dependency tree
class DepTree:
    def __init__(self, filename):
        self.parse_xml(filename)

    # parses xml and returns a dependency tree
    def parse_xml(self, filename):
        et = ElementTree.parse(filename).getroot()

        # check root attribute
        if et.tag != 'install':
            raise RuntimeError("'%s' is not an instool xml file" %filename)

        # maps name->new dep
        self.deps = {}
        # maps yet-to-be-resolved-dep->dep asking for it
        self.dep_in_me = {}

        for raw_dep in et.findall('dep'):
            # build this dep
            dep_name = raw_dep.get('name')
            new_dep = Dep(dep_name)

            # build conds
            for cond in raw_dep.findall('cond'):
                name = cond.get('name')
                dir = cond.get('dir')
                complement = cond.get('complement')
                if not complement is None:
                    complement = complement.lower() == 'true'
                raw_cond = cond.text
                new_dep.add_cond(name, raw_cond, dir, complement)

            # build bashes
            for bash in raw_dep.findall('run'):
                dir = bash.get('dir')
                cond = bash.get('cond')
                if not cond is None:
                    if cond in new_dep.conds:
                        cond = new_dep.conds[cond]
                    else:
                        raise RuntimeError("Cannot resolve cond: %s" % cond)
                raw_bash = bash.text
                new_dep.add_bash(raw_bash, cond, dir)
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

    # compiles deps to a bash file
    def to_file(self, filename, ask=False):
        # open file and get dep list
        file = open(filename, 'w')
        file.write("INST_DIR=\"%s\"\n" %MY_DIR)
        l = self.__extract__()
        # write deps to file
        prev_dir = MY_DIR
        pref = 0
        for dep in l:
            # add question, in case of -ask
            if ask:
                prev_dir = "" # in case of -ask, always 'cd' before executing dep
                file.write("%swhile true; do\n"
                           "%sread -p \"Should I install '%s'? [y/n] \" yn\n"
                           "%scase $yn in\n"
                           "%s[Yy]* ) echo;\n" %(PREF_STR*pref, PREF_STR*(pref+1), dep.name,
                                                         PREF_STR*(pref+1), PREF_STR*(pref+2)))
                pref += 3
            prev_dir = dep.to_file(file, prev_dir, pref)
            if ask:
                pref -= 3
                file.write("\n%sbreak;;\n"
                           "%s[Nn]* ) break;;\n"
                           "%s    * ) echo;;\n"
                           "%sesac\n"
                           "%sdone\n" %(PREF_STR*(pref+3), PREF_STR*(pref+2), PREF_STR*(pref+2),
                                        PREF_STR*(pref+1), PREF_STR*pref))
        # close file
        file.close()

    # returns a list of deps to be executed by order
    def __extract__(self):
        l = []
        for root in self.roots:
            self.__rec_extract__(root, l)
        return l

    # helper recursive, to be used by __extract__()
    def __rec_extract__(self, dep, l):
        for prev_dep in dep.get_deps():
            # make sure no dep is printed more then once
            if not prev_dep in l:
                self.__rec_extract__(prev_dep, l)
        l.append(dep)

    def tree(self):
        print("[Dependency Tree]")
        l = []
        for root in self.roots:
            self.__rec_tree__(root, 0, l)

    def __rec_tree__(self, dep, d, l):
        print("%s%s" %("    "*d, dep.name))
        if dep.name in l:
            if len(dep.get_deps())>0:
                print("%s..." %("    "*(d+1)))
        else:
            l.append(dep.name)
            for prev_dep in dep.get_deps():
                self.__rec_tree__(prev_dep, d+1, l)

    # list executed deps in-order
    def list(self):
        print("[Installation Order]")
        for i, dep in enumerate(self.__extract__()):
            print("%3i   %s" % (i+1, dep.name))

dt = DepTree(INSTALL_XML)
if NO_INST:
    for dep in NO_INST_LIST:
        if dep==dt.roots[0].name:
            raise RuntimeError("Cannot cut root branch installation")
        dt.cut_branch(dep)

if ONLY_INST:
    dt.roots = []
    for root_name in ONLY_INST_LIST:
        if not root_name in dt.deps:
            raise RuntimeError("Cannot resolve root: %s" % root_name)
        dt.roots.append(dt.deps[root_name])

if NO_PREV:
    dt.roots = []
    for root_name in NO_PREV_LIST:
        if not root_name in dt.deps:
            raise RuntimeError("Cannot resolve root: %s" % root_name)
        dt.deps[root_name].clear_deps()
        dt.roots.append(dt.deps[root_name])

if LIST:
    dt.list()
elif TREE:
    dt.tree()
else:
    dt.to_file(COMPILED_BASH, ASK)
    if not COMPILE:
        # chmod COMPILED_BASH u+x
        curr_stat = stat(COMPILED_BASH).st_mode
        chmod(COMPILED_BASH, curr_stat | S_IEXEC)
        # run bash
        ex_code = call("bash ./%s" %COMPILED_BASH, shell=True)
        if ex_code!=0:
            print("Installation was not successful; Something went wrong...")
        else:
            print("Installation was successful!")
        # delete bash
        remove(COMPILED_BASH)
