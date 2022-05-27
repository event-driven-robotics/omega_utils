import yarp
from time import sleep, time
import numpy as np


class OmegaControl:

    def __init__(self):
        #   Initialize yarp
        while not yarp.Network.checkNetwork():
            print("YARP network is not up. Checking again in 2 seconds.")
            sleep(2)

        yarp.Network.init()
        self.rpc_client = yarp.RpcClient()
        self.start_pos_x = self.start_pos_y = self.start_pos_z = None

    def start(self):
        self.rpc_client.open('/server-omega/rpc:o')
        if not yarp.NetworkBase.connect('/server-omega/rpc:o', '/yarp-omega3-server/rpc:i'):
            print("Failed to connect to server")
            return False

        # setting up connection for recieving position stream
        self.robot_state_stream = yarp.BufferedPortVector()
        self.robot_state_stream.open("/robot_state:o")
        yarp.Network.connect("/yarp-omega3-server/robot_state:o", "/robot_state:o")
        if not yarp.Network.connect("/yarp-omega3-server/robot_state:o", "/robot_state:o"):
            print("Failed to connect to yarp port for robot state")
            return False
        return True

    def send_position_reference(self, x, y, z, wait_for_motion_done=True, tolerance=0.001, timeout=5):

        cmd = yarp.Bottle()
        reply = yarp.Bottle()

        cmd.addString('set_position')
        cmd.addDouble(x)
        cmd.addDouble(y)
        cmd.addDouble(z)

        self.rpc_client.write(cmd, reply)
        if wait_for_motion_done:
            start_time = time()
            while True:
                current_position = self.yarp_vector_to_numpy(self.robot_state_stream.read(True))[:3]
                if np.linalg.norm(current_position - [x, y, z]) < tolerance or time() - start_time > timeout:
                    break

    def send_position_reference_w_tracking(self, x, y, z):

        cmd = yarp.Bottle()
        reply = yarp.Bottle()

        cmd.addString('track_position')
        cmd.addDouble(x)
        cmd.addDouble(y)
        cmd.addDouble(z)

        self.rpc_client.write(cmd, reply)

    def set_position_move_param(self, amax, vmax, jerk):

        cmd = yarp.Bottle()
        reply = yarp.Bottle()

        cmd.addString('set_position_move_param')
        cmd.addDouble(amax)  # m/s^2
        cmd.addDouble(vmax)  # m/s
        cmd.addDouble(jerk)  # m/s^3

        self.rpc_client.write(cmd, reply)

    def set_position_track_param(self, amax, vmax, jerk):

        cmd = yarp.Bottle()
        reply = yarp.Bottle()

        cmd.addString('set_position_track_param')
        cmd.addDouble(amax)
        cmd.addDouble(vmax)
        cmd.addDouble(jerk)

        self.rpc_client.write(cmd, reply)

    def send_stop(self):

        cmd = yarp.Bottle()
        reply = yarp.Bottle()

        cmd.addString('stop')

        self.rpc_client.write(cmd, reply)

    def close(self):
        self.send_stop()
        self.rpc_client.close()

    def get_robot_position(self):
        return self.get_robot_state()[0:3]  # x, y, z coordinates

    def get_robot_forces(self):
        return self.get_robot_state()[6:9]  # x, y, z coordinates

    def get_robot_state(self):
        read = self.robot_state_stream.read()
        return self.yarp_vector_to_numpy(read)

    def set_home(self):
        self.send_stop()
        print('Move fingertip to top left nodule')
        input('Press Enter when the fingertip is in position')
        print('Stand Still!')
        now = time()
        accumulated_positions = []
        while time() - now < 2:
            accumulated_positions.append(self.yarp_vector_to_numpy(self.robot_state_stream.read(True))[:3])
        print('Now I know where my place is, my lord')
        self.start_pos_x, self.start_pos_y, self.start_pos_z, = np.array(accumulated_positions).mean(0)
        self.home()
        
    def home(self):
        if self.start_pos_x is not None:
            print('Going home!')
            self.send_position_reference(self.start_pos_x, self.start_pos_y, self.start_pos_z)
        else:
            print('Set home position first')

    def yarp_vector_to_numpy(self, vector):
        return np.array([vector[i] for i in range(vector.size())])
