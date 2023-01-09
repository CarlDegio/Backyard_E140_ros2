import socket

class BackYardGripper:
    def __init__(self,host="192.168.137.9",port=9999):
        self.host, self.port = host, port  # 夹爪 IP 和 TCP/IP 服务器端口号
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.host, self.port))
        except Exception as e:
            print(e)

    def moveto(self, position, speed, acceleration, torque, tolerance=90, waitflag=True):
        cmd = "moveTo"
        cmd_args = cmd + "({0},{1},{2},{3},{4},{5})".format(position, speed, acceleration, torque, tolerance,waitflag) + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return int(str(self.sock.recv(1024), "utf-8"))

    def calibrate_gripper(self):
        cmd_args = "calibrateGripper()" + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return int(str(self.sock.recv(1024), "utf-8"))

    def get_status(self):
        cmd_args = "getStatus()" + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return str(self.sock.recv(1024), "utf-8")

    def get_calibrated(self):
        cmd_args = "getCalibrated()" + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return int(str(self.sock.recv(1024), "utf-8"))

    def shutdown(self):
        cmd_args = "shutdown()" + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return int(str(self.sock.recv(1024), "utf-8"))

    def restart(self):
        cmd_args = "restart()" + "\n"
        self.sock.sendall(bytes(cmd_args, "utf-8"))
        return int(str(self.sock.recv(1024), "utf-8"))

if __name__ == "__main__":
    gripper=BackYardGripper()
