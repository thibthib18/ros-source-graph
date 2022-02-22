
import unittest
from test.test_config import config
from src.log_parser import parse_log, parse_line


class TestLogParser(unittest.TestCase):
    def test_parse_item(self):
        parse_log(config)

    def test_parse_line(self):
        line = './test/logs/latest/backend-manual_ips-24.log:15:[rosout][ERROR] 2022-02-21 18:05:36,305: {"prefix": "ROS_SOURCE_GRAPH_PREFIX", "resource": "topic", "type": "producer","file": "/home/sv/main/backend/ui_components/src/ui_components/manual_ips.py", "line": "23", "resolved_name": "/backend/manual_ips/" }'
        data = parse_line(line)
        expected_data = {
            'prefix': "ROS_SOURCE_GRAPH_PREFIX",
            'resource': "topic",
            'type': "producer",
            'file': "/home/sv/main/backend/ui_components/src/ui_components/manual_ips.py",
            'line': "23",
            'resolved_name': "/backend/manual_ips/",
        }
        self.assertEqual(data, expected_data)
