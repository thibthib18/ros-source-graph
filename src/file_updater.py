import re
from typing import List
from src.common import ResourceConfig, Match

PREFIX = 'ROS_SOURCE_GRAPH_PREFIX'


def update_file(matches: List[Match]) -> None:
    sort_bottom_up(matches)
    config = matches[0].config
    insert_code_after_matches(config, matches)


def sort_bottom_up(matches: List[Match]):
    matches.sort(key=lambda match: match.line_number, reverse=True)


def insert_code_after_matches(config: ResourceConfig, matches: List[Match]) -> None:
    raw_lines_to_append: List[str] = config.insert_code_after
    for match in matches:
        lines_to_append = format_lines_to_append(
            raw_lines_to_append, match)
        insert_lines(match.file_path, match.line_number, lines_to_append)


def insert_lines(file_name: str, line_number: int, lines_to_append: List[str]) -> None:
    with open(file_name, "r") as f:
        contents = f.readlines()
    indent = get_whitespace(contents[line_number-1])
    indented_lines_to_append = list(map(
        lambda line: indent+line+'\n', lines_to_append))

    for i in range(len(indented_lines_to_append)):
        contents.insert(line_number + i, indented_lines_to_append[i])
    with open(file_name, "w") as f:
        f.writelines(contents)


def format_lines_to_append(lines: List[str],  match: Match) -> List[str]:
    formatted_lines = []
    config = match.config
    if match.node_handle is None:
        formatted_lines.append(lines[0].format(match.name))
        formatted_lines.append(lines[1].format(
            PREFIX, config.resource_type, config.role, match.file_path, match.line_number, match.name))
    else:
        formatted_lines.append(lines[0])
        formatted_lines.append(lines[1].format(match.node_handle, match.name))
        formatted_lines.append(lines[2].format(
            PREFIX, config.resource_type, config.role, match.file_path, match.line_number, match.name))
        formatted_lines.append(lines[3])
    return formatted_lines


def get_whitespace(line: str) -> str:
    regex = r'^(\s*)\w.*'
    m = re.search(regex, line)
    if m is None:
        raise ValueError('No match for whitespace.')
    return m.group(1)
