import socket
import hashlib
import threading

# robot_address = ("192.168.0.186", 9979)

class UDPClient:
    def __init__(self, address):
        self.address = address
        self.private_key_1 = self.compute_md5("AROC-PL")[0:10]
        self.private_key_2 = self.compute_md5("Bismillah")[0:15]
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client.bind(self.address)

    @staticmethod
    def compute_md5(data):
        md5_hash = hashlib.md5(data.encode()).hexdigest()
        return md5_hash

    @staticmethod
    def decrypt_key(input_string, string_to_remove):
        new_string = input_string.replace(string_to_remove, "")
        return new_string

    def close(self):
        self.client.close()

    def decrpyt_data(self, data):
        data = data.decode('utf-8')
        decrypted_key_1 = self.decrypt_key(data, self.private_key_1)
        decrypted_data = self.decrypt_key(decrypted_key_1, self.private_key_2)
        STATE_LABELS = {
            "0": 'INITIAL',
            "1": 'READY',
            "2": 'SET',
            "3": 'PLAY',
            "4": 'FINISH'
        }
        state_data = STATE_LABELS.get(decrypted_data, 'UNKNOWN')
        return state_data

    def run(self):
        data, addr = self.client.recvfrom(1024)
        return self.decrpyt_data(data)

    # def start(self):
    #     thread = threading.Thread(target=self.run)
    #     thread.start()


# def main():
#     udp_client = UDPClient(robot_address)
#     udp_client.run()
#
# if __name__ == "__main__":
#     main()
