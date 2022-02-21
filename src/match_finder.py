import subprocess
from typing import List, Optional, Tuple
import re
from src.common import ResourceConfig, Match


def find_matches(config: ResourceConfig) -> List[Match]:
    # grep command
    cmd = fr'{config.grep_command} "{config.regex}" {config.target_dir}'
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    p.wait()
    (out, _) = p.communicate()
    # decode and filter out empty strings
    filesLines = list(filter(None, out.decode('utf-8').split('\n')))
    matches = []
    # parse output
    for fileLine in filesLines:
        fileLineNumber, line = fileLine.split(' ', 1)
        file, line_number = fileLineNumber.split(':')[0:2]
        name, node_handle = parse_line(line, config.regex, config.language)
        match = Match(config, file, int(line_number), name, node_handle)
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
