
from src import main
from test.test_config import config
import unittest


class TestCodeUpdater(unittest.TestCase):
    def test_get_find_matches_command(self):
        rmf = main.ResourceMatchFinder(
            config, 'cpp', 'topic', 'producer')
        print(rmf.get_find_matches_command())

    def test_find_matches(self):
        rmf = main.ResourceMatchFinder(
            config, 'cpp', 'topic', 'producer')
        #regex = r'(\w*)\.advertise[^S].*\(([^,]*).*\)'
        matches = main.find_matching_file_lines(
            rmf.get_find_matches_command(), rmf.regex, rmf.language)
        expected_matches = [{'file': './test/test_directory/another_roscpp_node.cpp', 'line_number': 25, 'node_handle': 'n', 'name': '"chatter"'},
                            {'file': './test/test_directory/a_roscpp_node.cpp', 'line_number': 25, 'node_handle': 'n', 'name': '"chatter22"'}]
        self.assertEqual(matches, expected_matches)

    def test_insert_code_after_match(self):
        file_name = './test/test_directory/a_rospy_node.py'
        line_number = 23
        lines_to_append = ['rospy.logerr("whatever")', '# dont worry']
        main.insert_lines(file_name, line_number, lines_to_append)

    def test_get_whitespace(self):
        ws = main.get_whitespace('    rate = rospy.Rate(10)')
        self.assertEqual(ws, '    ')

    def test_format_lines_to_append(self):
        match = {'file': './test/test_directory/another_roscpp_node.cpp',
                 'line_number': 25, 'node_handle': 'n', 'name': '"chatter"'}
        conf = {'language': 'cpp', 'resource': 'topics', 'type': 'advertiser'}
        prefix = 'ROS_SOURCE_GRAPH_PREFIX'
        lines = config['cpp']['insert_code_after']
        formatted_lines = main.format_lines_to_append(
            prefix, lines, **match, **conf)
        print(formatted_lines)
