#!/usr/bin/python

import sys
import os
PKG = os.path.split(os.getcwd())[1]
print "Creating pydev project for package '%s'"%PKG
import roslib; roslib.load_manifest(PKG)

pathlist = "\n".join(["<path>%s</path>"%path for path in sys.path if os.path.exists(path)])

pydev_project= '''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<?eclipse-pydev version="1.0"?>

<pydev_project>
<pydev_property name="org.python.pydev.PYTHON_PROJECT_INTERPRETER">Default</pydev_property>
<pydev_property name="org.python.pydev.PYTHON_PROJECT_VERSION">python 2.6</pydev_property>
<pydev_pathproperty name="org.python.pydev.PROJECT_EXTERNAL_SOURCE_PATH">
%s
</pydev_pathproperty>
</pydev_project>
'''%pathlist

print "Writing .pydevproject, adding %d modules"%len(sys.path)
f = open(".pydevproject","w")
f.write(pydev_project)
f.close()
