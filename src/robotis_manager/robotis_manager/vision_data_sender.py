import os
import sys


class DataHandler:
    def __init__(self) -> None:
        pass

    def write(self, file, path):
        with open(path, "w") as f:
            f.write(file)

    def read(self, path):
        with open(path, "r") as f:
            return f.read()

    def append(self, file, path):
        with open(path, "a+") as f:
            f.write(file)

    def remove(self, path):
        if os.path.exists(path):
            os.remove(path)

    def remove_dir(self, path):
        if os.path.exists(path):
            os.rmdir(path)
