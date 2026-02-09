import subprocess


def check_ssh_connections():
    result = subprocess.run(["who"], stdout=subprocess.PIPE)
    output = result.stdout.decode("utf-8")
    ssh_connections = [line for line in output.split("\n") if "pts/" in line]
    return len(ssh_connections) > 0  # bool


if __name__ == "__main__":
    if check_ssh_connections():
        print("There are active SSH connections.")
    else:
        print("No active SSH connections.")
