import subprocess
from typing import List, Optional, Tuple
import re
from src.common import ResourceConfig, Match, grep


def find_matches(config: ResourceConfig) -> List[Match]:
    findings = grep(config.grep_command, config.regex, config.target_dir)
    matches = []
    # parse output
    for finding in findings:
        file, line_number, line_content = finding
        name, node_handle = parse_line(
            line_content, config.regex, config.language)
        match = Match(config, file, line_number, name, node_handle)
        matches.append(match)
    return matches


def parse_line(line, regex, language) -> Tuple[str, Optional[str]]:
    m = re.search(regex, line)
    if m is None:
        raise ValueError(f'Line {line} does match regex {regex}')
    if language == 'python':
        return m.group(1), None
    # cpp
    return m.group(2),  m.group(1)
