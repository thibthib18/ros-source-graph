from src import match_finder
from src.common import ResourceConfig, Match
from test.test_config import config as test_config
import unittest


class TestMatchFinder(unittest.TestCase):

    def test_find_matches(self):
        config = ResourceConfig(
            test_config, 'cpp', 'topic', 'producer')
        matches = match_finder.find_matches(config)
        expected_matches = [Match(config, './test/test_directory/another_roscpp_node.cpp', 25, '"chatter"', 'n'),
                            Match(config, './test/test_directory/a_roscpp_node.cpp', 25, '"chatter22"', 'n')]
        for i in range(len(expected_matches)):
            self.assertEqual(matches[i], expected_matches[i])
