import subprocess
import os
import vcs_base

class GITClient(vcs_base.VCSClientBase):
    def get_url(self):
        """
        @return: GIT URL of the directory path (output of git info command), or None if it cannot be determined
        """
        if self.detect_presence():
            output = subprocess.Popen(["git", "config",  "--get", "remote.origin.url"], cwd=self._path, stdout=subprocess.PIPE).communicate()[0]
            return output.rstrip()
        return None

    def detect_presence(self):
        return self.path_exists() and os.path.isdir(os.path.join(self._path, '.git'))


    def checkout(self, url, version=''):
        if self.path_exists():
            print >>sys.stderr, "Error: cannnot checkout into existing directory"
            return False
            
        cmd = "git clone %s %s"%(url, self._path)
        if not subprocess.check_call(cmd, shell=True) == 0:
            return False
        cmd = "git checkout %s -b rosinstall"%(version)
        if not subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return False
        return True

    def update(self, version=''):
        if not self.detect_presence():
            return False
        cmd = "git checkout master"
        if not subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return False
        cmd = "git fetch"
        if not subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return False
        cmd = "git branch -D rosinstall"
        if not subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            pass # OK to fail return False
        cmd = "git checkout %s -b rosinstall"%(version)
        if not subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return False
        return True
        
    def get_vcs_type_name(self):
        return 'git'
