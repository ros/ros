import subprocess
import os
import vcs_base

class BZRClient(vcs_base.VCSClientBase):
    def get_url(self):
        """
        @return: BZR URL of the directory path (output of bzr info command), or None if it cannot be determined
        """
        if self.detect_presence():
            output = subprocess.Popen(['bzr', 'info', self._path], stdout=subprocess.PIPE).communicate()[0]
            matches = [l for l in output.split('\n') if l.startswith('  checkout of branch:')]
            if matches:
                return matches[0][22:]
        return None

    def detect_presence(self):
        return self.path_exists() and os.path.isdir(os.path.join(self._path, '.bzr'))


    def checkout(self, url, version=''):
        if self.path_exists():
            print >>sys.stderr, "Error: cannnot checkout into existing directory"
            return False
            
        cmd = "bzr co %s %s %s"%(version, url, self._path)
        if subprocess.check_call(cmd, shell=True) == 0:
            return True
        return False

    def update(self, version=''):
        if not self.detect_presence():
            return False
        if not subprocess.check_call("bzr update", cwd=self._path, shell=True) == 0:
            return False
        cmd = "bzr revert %s"%(version)
        if subprocess.check_call(cmd, cwd=self._path, shell=True) == 0:
            return True
        return False
        
    def get_vcs_type_name(self):
        return 'bzr'
