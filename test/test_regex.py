from typing import List
import unittest
import re
from src.config import config

reg_adv_topic_cpp = config['cpp']['topics']['advertiser']['regex']
reg_sub_topic_cpp = config['cpp']['topics']['subscribers']['regex']
reg_adv_service_cpp = config['cpp']['services']['advertiser']['regex']
reg_call_service_cpp = config['cpp']['services']['callers']['regex']

reg_adv_topic_py = config['python']['topics']['advertiser']['regex']
reg_sub_topic_py = config['python']['topics']['subscribers']['regex']
reg_adv_service_py = config['python']['services']['advertiser']['regex']
reg_call_service_py = config['python']['services']['callers']['regex']


class TestRegex(unittest.TestCase):
    def print_match(self, m):
        if m is not None:
            print(m.group(1))
        else:
            print('No match')

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
