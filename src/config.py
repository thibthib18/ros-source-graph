config = {
    'target_dir': '/home/user/main',
    'compile_command': 'catkin_make',
    'launch_command': 'roslaunch launch_files main.launch',
    # ripgrep (rg) is much faster and respects .gitignore by default
    # Also include here a potential exclude pattern or other options
    'grep_command': 'grep --exclude-dir=node_modules -RnP',
    'cpp': {
        'insert_code_after': [
            '''std::string resolvedName = {0}.resolveName({1});''',
            '''ROS_ERROR_STREAM("{{'prefix': {0}, 'resource': {1}, 'type': {2}, 'file': {3} ,'line': {4}, 'name': {5}, 'resolved_name': " << resolvedName << "}}");'''],
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
            '''resolved_name = rospy.resolved_name({0})'''
            '''rospy.logerr(f'{{"prefix": {0}, "resource": {1}, "type": {2},"file": {3}, "line": {4},"name": {5}, "resolved_name": {{resolved_name}} }}')'''
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
