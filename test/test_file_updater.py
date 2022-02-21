from src import file_updater
from src.common import ResourceConfig, Match
from test.test_config import config as test_config
import unittest


class TestFileUpdater(unittest.TestCase):

    def test_insert_code_after_match(self):
        file_name = './test/test_directory/a_rospy_node.py'
        line_number = 23
        lines_to_append = ['rospy.logerr("whatever")', '# dont worry']
        file_updater.insert_lines(file_name, line_number, lines_to_append)

    def test_get_whitespace(self):
        ws = file_updater.get_whitespace('    rate = rospy.Rate(10)')
        self.assertEqual(ws, '    ')

    def test_format_lines_to_append(self):
        config = ResourceConfig(test_config, 'cpp', 'topics', 'advertiser')
        match = Match(
            config, './test/test_directory/another_roscpp_node.cpp', 25, '"chatter"', 'n')
        lines = config.insert_code_after
        formatted_lines = file_updater.format_lines_to_append(lines, match)
        print(formatted_lines)
