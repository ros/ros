import subprocess

####### Linux Helper Functions #####
def lsb_get_os():
    try:
        cmd = ['lsb_release', '-si']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_codename():
    try:
        cmd = ['lsb_release', '-sc']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None
    
def lsb_get_release_version():
    try:
        cmd = ['lsb_release', '-sr']
        pop = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (std_out, std_err) = pop.communicate()
        return std_out.strip()
    except:
        return None

