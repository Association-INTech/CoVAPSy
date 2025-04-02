import netifaces as ni #netifaces is abondened, so I use netifaces2
import subprocess

def get_ip(device='wlan0'):
    ip = ni.ifaddresses(device)[ni.AF_INET][0]['addr'] #note this only gets the first IP address of the interface. I can have sevral IP addresses
    return ip


def check_ssh_connections():
    result = subprocess.run(['who'], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8')
    ssh_connections = [line for line in output.split('\n') if 'pts/' in line]
    return len(ssh_connections) > 0 #bool
if __name__ == "__main__":
    if check_ssh_connections():
        print("There are active SSH connections.")
    else:
        print("No active SSH connections.")