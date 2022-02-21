from typing import Optional


class ResourceConfig(object):
    def __init__(self, config, language: str, resource_type: str, role: str) -> None:
        self.language = language
        self.resource_type = resource_type
        self.role = role
        self.config = config
        self.regex = config[language][resource_type][role]["regex"]
        self.grep_command = config['grep_command']
        self.target_dir = config['target_dir']
        self.insert_code_after = config[language]['insert_code_after']

    def __eq__(self, other) -> bool:
        return self.__dict__ == other.__dict__


class Match(object):
    def __init__(self, config: ResourceConfig, file_path: str, line_number: int, name: Optional[str], node_handle: Optional[str]) -> None:
        self.config = config
        self.line_number = line_number
        self.file_path = file_path
        self.name: Optional[str] = name
        self.node_handle: Optional[str] = node_handle

    def __eq__(self, other) -> bool:
        return self.__dict__ == other.__dict__

    def __str__(self):
        return str(self.__dict__)
