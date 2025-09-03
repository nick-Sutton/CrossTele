import yaml

class NatNetParser:
    def __init__(self):
        pass

    def parse_config_file(self, file_path):
        with open(file_path, 'r') as file:
            config_data = yaml.safe_load(file)

        return config_data
