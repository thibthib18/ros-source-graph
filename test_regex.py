import unittest
import re

# Regex explanation left to right
# Match any word (valid var name) as capture group 1; find .advertise; then whatever
# but no S (to exclude advertiseService); then (; then 2nd capture group whatever but no `,`;
# then one `,`; then whatever; then );
reg_adv_topic_cpp = r'(\w*)\.advertise[^S]*\(([^,]*),.*\)'
reg_adv_service_cpp = r'(\w*)\.advertiseService.*\(([^,]*),.*\)'

reg_adv_topic_py = r'rospy.Publisher\(([^,]*),.*\)'
reg_adv_service_py = r'(\w*)\.advertiseService.*\(([^,]*),.*\)'


class TestRegex(unittest.TestCase):
    def print_match(self, m):
        if m is not None:
            print(m.group(1))
        else:
            print('No match')

    def test_match_advertise_topic_cpp(self):
        valid_line = 'frontendPublisher = privateNodeHandle.advertise<sv_msgs::Float32Array>("", 1, true);'
        invalid_line = 'getService = privateNodeHandle.advertiseService("get", &Shot::getShotCallback, this);'
        self.assertIsNotNone(re.search(reg_adv_topic_cpp, valid_line))
        self.assertIsNone(re.search(reg_adv_topic_cpp, invalid_line))

    def test_find_handle_advertise_topic_cpp(self):
        valid_line = 'frontendPublisher = privateNodeHandle.advertise<sv_msgs::Float32Array>("get", 1, true);'
        match = re.search(reg_adv_topic_cpp, valid_line)
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), 'privateNodeHandle')
        # print(match.group(2))
        self.assertEqual(match.group(2), '"get"')


if __name__ == '__main__':
    unittest.main()
