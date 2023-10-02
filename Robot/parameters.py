import json


class JSONToClass:
    def __init__(self, JSON: dict = None):
        for key, value in JSON.items():
            if isinstance(value, dict): setattr(self, key, JSONToClass(value))
            else: setattr(self, key, value)


class Robot(JSONToClass):
    def __init__(self, filename: str = 'parameters.json'):
        with open(filename, 'r') as f:
            super().__init__(json.load(f))
