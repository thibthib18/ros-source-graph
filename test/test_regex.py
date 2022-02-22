from typing import List
import unittest
import re
from src.config import config
from src import match_finder

reg_adv_topic_cpp = config['cpp']['topic']['producer']['regex']
reg_sub_topic_cpp = config['cpp']['topic']['consumer']['regex']
reg_adv_service_cpp = config['cpp']['service']['producer']['regex']
reg_call_service_cpp = config['cpp']['service']['consumer']['regex']

reg_adv_topic_py = config['python']['topic']['producer']['regex']
reg_sub_topic_py = config['python']['topic']['consumer']['regex']
reg_adv_service_py = config['python']['service']['producer']['regex']
reg_call_service_py = config['python']['service']['consumer']['regex']

failinglines = [
    'nh.advertiseService("/backend/ptu_limits/" + axisToString(axis) + "/get", &LensLimits::getCallback, this);',
    'ptuStateWithPositionPublisher = nh().advertise<sv_msgs::PTUState_v2>("/debug/ptu_state/with_position", 1);',
    'lens_pub = nh().advertise<sv_msgs::LensState>("/driver/req_lens_state", 1);',
    '''lensCalibrationFeedbackSubscriber = nodeHandle.subscribe(
        "/calibration/zoom_mapping_calibration_node/feedback", 1, &LensCalibration::onLensCalibrationFeedback, this);''',
    '''rospy.Subscriber("/cinematography/start_action",
        sv_msgs.msg.CinematographyAction,
        self._action_callback,
        queue_size=5)'''
]

# Match group 1 only first arg
r = r'\w*\.advertiseService\((.*)(,.*)/U'  # needs ungreedy


class TestRegex(unittest.TestCase):
    def print_match(self, m):
        if m is not None:
            print(m.group(1))
        else:
            print('No match')

    def test_parse_multilines(self):
        line = failinglines[3]
        regex = reg_sub_topic_cpp
        result = match_finder.parse_line(line, regex, 'cpp')
        m = re.search(regex, line, re.MULTILINE)
        print(m.group(2))

    def test_parseline_cpp(self):
        line = 'fePublisher = privateNodeHandle.advertise<my_msgs::Float32Array>("get", 1, true);'
        regex = reg_adv_topic_cpp
        result = match_finder.parse_line(line, regex, 'cpp')
        self.assertEqual(result[0], '"get"')
        self.assertEqual(result[1], 'privateNodeHandle')

    def test_parseline_python(self):
        line = 'self.name_publisher = rospy.Publisher("~name", String, queue_size=1)'
        regex = reg_adv_topic_py
        result = match_finder.parse_line(line, regex, 'python')
        self.assertEqual(result['name'], '"~name"')

    def reg_match_line(self, reg: str,  line: str, groups: List[str]):
        match = re.search(reg, line)
        self.assertIsNotNone(match)
        for i in range(len(groups)):
            self.assertEqual(match.group(i+1), groups[i])

    def test_match_advertise_topic_cpp(self):
        invalid_line = 'getService = privateNodeHandle.advertiseService("get", &Mymsg::getCallback, this);'
        self.assertIsNone(re.search(reg_adv_topic_cpp, invalid_line))

        valid_line = 'fePublisher = privateNodeHandle.advertise<my_msgs::Float32Array>("get", 1, true);'
        self.reg_match_line(reg_adv_topic_cpp, valid_line,
                            groups=['privateNodeHandle', '"get"'])

    def test_match_advertise_topic_py(self):
        valid_line = 'self.name_publisher = rospy.Publisher("~name", String, queue_size=1)'
        match = re.search(reg_adv_topic_py, valid_line)
        self.reg_match_line(reg_adv_topic_py, valid_line,
                            groups=['"~name"'])

    def test_match_subscribe_topic_py(self):
        valid_line1 = 'rospy.Subscriber(publication[0], publication[1], callback)'
        valid_line2 = 'topicSubscriber = rospy.Subscriber("/some/topic", std_msgs.msg.Bool, self.callback, queue_size=1)'
        self.reg_match_line(reg_sub_topic_py, valid_line1,
                            groups=['publication[0]'])
        self.reg_match_line(reg_sub_topic_py, valid_line2,
                            groups=['"/some/topic"'])

    def test_call_service_cpp(self):
        valid_line1 = 'ros::ServiceClient client = nh.serviceClient<my_package::Foo>("my_service_name");'
        self.reg_match_line(reg_call_service_cpp, valid_line1,
                            groups=['nh', '"my_service_name"'])

    def test_subscribe_topic_cpp(self):
        valid_line1 = 'ros::Subscriber sub = nh.subscribe("my_topic", 1, callback);'
        self.reg_match_line(reg_sub_topic_cpp, valid_line1,
                            groups=['nh', '"my_topic"'])


if __name__ == '__main__':
    unittest.main()
