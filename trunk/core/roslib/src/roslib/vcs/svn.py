import subprocess
import os
import vcs_base

class SVNClient(vcs_base.VCSClientBase):
    def get_url(self):
        """
        @return: SVN URL of the directory path (output of svn info command), or None if it cannot be determined
        """
        if self.detect_presence():
            output = subprocess.Popen(['svn', 'info', self._path], stdout=subprocess.PIPE).communicate()[0]
            matches = [l for l in output.split('\n') if l.startswith('URL: ')]
            if matches:
                return matches[0][5:]
        return None

    def detect_presence(self):
        return self.path_exists() and os.path.isdir(os.path.join(self._path, '.svn'))

    def exists(self, url):
        """
        @return: True if url exists in repo
        """
        cmd = ['svn', 'info', url]
        output = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        return bool(output[0])

    def checkout(self, url, version=''):
        if self.path_exists():
            print >>sys.stderr, "Error: cannot checkout into existing directory"
            return False
            
        cmd = "svn co %s %s %s"%(version, url, self._path)
        if subprocess.check_call(cmd, shell=True) == 0:
            return True
        return False

    def update(self, version=''):
        if not self.detect_presence():
            return False
        cmd = "svn up %s %s"%(version, self._path)
        if subprocess.check_call(cmd, shell=True) == 0:
            return True
        return False
        
    def get_vcs_type_name(self):
        return 'svn'
