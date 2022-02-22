
from src.config import config


test_config = config
test_config['target_dir'] = './test/test_directory'
test_config['log_dir'] = './test/logs/latest'
test_config['source_graph_location'] = './test/ros_source_graph.json'
