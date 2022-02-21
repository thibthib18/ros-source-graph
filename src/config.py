config = {
    # Where to read the ROS log
    'log_location': '~/.ros/log/latest/rosout.log',
    # Where to save the generated source graph
    'source_graph_location': '~/.ros/ros_source_graph.json',
    # Location of the codebase
    'target_dir': '~/Projects/ros-source-graph/test/test_directory',
    # Command to compile the project
    'compile_command': 'catkin_make',
    # Command to launch the project
    'launch_command': 'roslaunch launch_files main.launch',
    # Grep command to find files advertising services/topics
    # include here a potential exclude pattern or other options
    # ripgrep (rg) is much faster and respects .gitignore by default
    'grep_command': 'grep --exclude-dir=node_modules -RnP',
    'cpp': {
        'insert_code_after': ['{',
                              '''std::string resolvedName = {0}.resolveName({1});''',
                              # `"` -> `\"` -> `\\\"`, quotes are escaped twice: once for python format to fill the fields, another one for the C++ log
                              '''ROS_ERROR_STREAM("{{\\\"prefix\\\": \\\"{0}\\\", \\\"resource\\\": \\\"{1}\\\", \\\"type\\\": \\\"{2}\\\", \\\"file\\\": \\\"{3}\\\" ,\\\"line\\\": {4}, \\\"resolved_name\\\": \\\""<< resolvedName <<"\\\"}}");''',
                              '}'],
        'service': {
            'producer': {
                'regex': r'(\w*)\.advertiseService.*\(([^,]*),.*\)',
            },
            'consumer': {
                'regex': r'(\w*)\.serviceClient.*\(([^,]*).*\)',
            },
        },
        'topic': {
            'producer': {
                # Regex explanation left to right
                # Match any word (valid var name) as capture group 1; find .advertise; then whatever
                # but no S (to exclude advertiseService and still catch the <msg> template); then (; then 2nd capture group whatever but no `,`;
                # then whatever; then );
                'regex': r'(\w*)\.advertise[^S].*\(([^,]*).*\)',
            },
            'consumer': {
                'regex': r'(\w*)\.subscribe*\(([^,]*),.*\)',
            },
        }
    },
    'python': {
        'insert_code_after': [
            '''resolved_name = rospy.resolve_name({0})''',
            '''rospy.logerr(f'{{{{"prefix": \"{0}\", "resource": \"{1}\", "type": \"{2}\","file": \"{3}\", "line": {4}, "resolved_name": \"{{resolved_name}}\" }}}}')'''
        ],
        'service': {
            'producer': {
                'regex': r'rospy.Service\(([^,]*),.*\)',
            },
            'consumer': {
                'regex': r'rospy.ServiceProxy\(([^,]*),.*\)',
            },
        },
        'topic': {
            'producer': {
                'regex': r'rospy.Publisher\(([^,]*),.*\)',
            },
            'consumer': {
                'regex': r'rospy.Subscriber\(([^,]*),.*\)',
            }
        }
    }
}
