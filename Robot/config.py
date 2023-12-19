import json
from micropython import const


class JSONToClass:
    """
    Converts a JSON object to a class object.
    Parameters:
        JSON: dict
    """
    def __init__(self, JSON: dict = None):
        for key, value in JSON.items():
            if isinstance(value, dict): setattr(self, key, JSONToClass(value))
            else: setattr(self, key, const(value))


class config(JSONToClass):
    """
    Config class
    Used to load the config file.
    Parameters:
        filename: str
    """
    def __init__(self, filename: str = '/home/robot/Robot/config.json'):
        with open(filename, 'r') as f:
            super().__init__(json.load(f))

    def __repr__(self):
        return str(self.__dict__)
