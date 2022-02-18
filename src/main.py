import subprocess
from typing import List, Optional
import re

# for each language
# for each topic(sub+adv), srv (adv+call)
# find all matching file+line
# for each match
# append the lines

# Literals are only python 3.8
#Language = Literal['cpp', 'python']
#ResourceType = Literal['service', 'topic']
# tried to find a language that fits both topic adv/sub and service adv/client
#Role = Literal['producer', 'consumer']


class ResourceMatchFinder:
    def __init__(self, config, language: str, resource_type: str, role: str) -> None:
        self.language = language
        self.resource_type = resource_type
        self.role = role
        self.config = config
        self.regex = config[self.language][self.resource_type][self.role]["regex"]

    def prepare_codebase(self):
        cmd = self.get_find_matches_command()
        matches = self.find_matching_file_lines(cmd, self.regex, self.language)
        self.insert_code_after_matches(matches)

    def get_find_matches_command(self):
        grep_command = self.config['grep_command']
        target_dir = self.config['target_dir']
        return fr'{grep_command} "{self.regex}" {target_dir}'

    def insert_code_after_matches(self, matches):
        PREFIX = 'ROS_SOURCE_GRAPH_PREFIX'
        for match in matches:
            raw_lines_to_append: List[str] = self.config[self.language]['insert_code_after']
            lines_to_append = format_lines_to_append(
                PREFIX, raw_lines_to_append, self.language, self.resource_type, self.role, **match)
            insert_lines(match['file'], match['line_number'], lines_to_append)


def find_matching_file_lines(cmd, regex, language):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    p.wait()
    (out, _) = p.communicate()
    # decode and filter out empty strings
    filesLines = list(filter(None, out.decode('utf-8').split('\n')))
    matches = []
    for fileLine in filesLines:
        fileLineNumber, line = fileLine.split(' ', 1)
        file, line_number = fileLineNumber.split(':')[0:2]
        names = parse_line(line, regex, language)
        matches.append({'file': file, 'line_number': int(line_number), **names})
    return matches


def parse_line(line, regex, language):
    m = re.search(regex, line)
    if m is None:
        return {}
    if language == 'python':
        return {'name': m.group(1)}
    # cpp
    return {'node_handle': m.group(1), 'name': m.group(2)}


def insert_lines(file_name, line_number, lines_to_append):
    with open(file_name, "r") as f:
        contents = f.readlines()
    indent = get_whitespace(contents[line_number-1])
    indented_lines_to_append = list(map(
        lambda line: indent+line+'\n', lines_to_append))

    for i in range(len(indented_lines_to_append)):
        contents.insert(line_number + i, indented_lines_to_append[i])
    with open(file_name, "w") as f:
        f.writelines(contents)


def get_whitespace(line):
    regex = r'^(\s*)\w.*'
    m = re.search(regex, line)
    if m is None:
        return
    return m.group(1)


def format_lines_to_append(prefix, lines, language, resource, type, file, line_number, name, node_handle: Optional[str]):
    formatted_lines = []
    if language == 'python':
        formatted_lines.append(lines[0].format(name))
        formatted_lines.append(lines[1].format(
            prefix, resource, type, file, line_number, name))
    else:  # cpp
        formatted_lines.append(lines[0].format(node_handle, name))
        formatted_lines.append(lines[1].format(
            prefix, resource, type, file, line_number, name))
    return formatted_lines
