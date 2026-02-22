#!/usr/bin/python
import socket
import construct
import time
from robotis_manager.robocup_game_control_data import RoboCupGameControlData, GAMECONTROLLER_DATA_PORT


class Receive:
    class Com:
        gameState = None
        previousState = None


class GameControllerListener:
    def __init__(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,
                               1)  # SO_REUSEADDR instead of SO_REUSEPORT to work while TCM is running
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client.bind(('', GAMECONTROLLER_DATA_PORT))
        self.addr = None
        self.client.settimeout(2)
        # self.data = None

    def receive(self):
        try:
            Receive.Com.gameState, self.addr = self.client.recvfrom(1024)
            parsed = RoboCupGameControlData.parse(Receive.Com.gameState)
            Receive.Com.gameState = parsed.state
            # print("From : {}".format(self.addr))
            STATE_LABELS = {
                0: 'INITIAL',
                1: 'READY',
                2: 'SET',
                3: 'PLAY',
                4: 'FINISH'
            }
            if Receive.Com.gameState != Receive.Com.previousState:
                Receive.Com.previousState = Receive.Com.gameState
                Receive.Com.gameState = STATE_LABELS.get(Receive.Com.gameState, 'UNKNOWN')
                print("GameController Data Enabled")
                print(Receive.Com.gameState)
                return Receive.Com.gameState
        except socket.timeout as e:
            time.sleep(3)
            print("GameController Disabled, Check Connection")
            # print("Error: Data GameController tidak bisa diterima. Wi-Fi ne gak konek paling.")
        except construct.core.ConstError:
            pass

    def close(self):
        self.client.close()
# def signal_handler(sig, frame):
#     # Penanganan sinyal SIGINT
#     print("Ctrl+C terdeteksi. Menghentikan program...")
#     if 'listener' in globals():
#         listener.close()
#     sys.exit(0)
