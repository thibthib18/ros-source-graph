
import os
from typing import Dict, Optional
from src.common import grep, PREFIX
import re
import json


def parse_log(config: Dict):
    log_dir = config['log_dir']
    ignore_files = ['master.log', 'rosout.log', 'rosout-1-stdout.log']
    findings = grep(config['grep_command'], PREFIX, log_dir)
    items = []
    for finding in findings:
        file, _, line_content = finding
        # TODO: ignore_files directly in grep not later here
        if file in ignore_files:
            continue
        data = parse_line(line_content)
        if data is not None:
            items.append(data)
    source_graph = get_source_graph(items)
    print('services:')
    print(len(source_graph['service'].keys()))
    print('topics:')
    print(len(source_graph['topic'].keys()))
    with open(config['source_graph_location'], 'w') as f:
        string = json.dumps(
            source_graph, default=lambda o: o.__dict__, sort_keys=True, indent=2)
        f.write(string)


def get_source_graph(items):
    # source_graph = {
    #     'service': {
    #         '/mysrv': {
    #             'producers': [{'file': '/home/ok.py', 'line': 21}],
    #             'consumers': [{'file': '/dir/sub.cpp', 'line': 38}]
    #         }}
    # }
    source_graph = {'service': {}, 'topic': {}}
    print(len(items))
    for item in items:
        resource_type = item['resource']
        resolved_name = item['resolved_name']
        role = item['type']
        file = item['file']
        line = item['line']
        if resolved_name not in source_graph[resource_type]:
            print(f'adding default for {resolved_name}')
            source_graph[resource_type][resolved_name] = {
                'producer': [], 'consumer': []}
        source_graph[resource_type][resolved_name][role].append(
            {'file': file, 'line': line})
    return source_graph


def parse_line(line_content) -> Optional[Dict]:
    r = r'\[rosout\]\[ERROR\][^\{]*(.*)'
    m = re.search(r, line_content)
    if m is not None:
        return json.loads(m.group(1))
