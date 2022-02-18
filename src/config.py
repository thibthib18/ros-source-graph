# from typing import TypedDict


# class ResourceConfig(TypedDict):
#     regex: str


# class ServiceConfig(TypedDict):
#     advertiser: ResourceConfig
#     callers: ResourceConfig


# class TopicConfig(TypedDict):
#     advertiser: ResourceConfig
#     subscribers: ResourceConfig


# class LanguageConfig(TypedDict):
#     services: ServiceConfig
#     topics: TopicConfig
#     insert_code_after: str


# class Config(TypedDict):
#     target_dir: str
#     compile_command: str
#     launch_command: str
#     grep_command: str
#     cpp: LanguageConfig
#     python: LanguageConfig


config = {
    'target_dir': '/home/user/main',
    'compile_command': 'catkin_make',
    'launch_command': 'roslaunch launch_files main.launch',
    # ripgrep (rg) is much faster and respects .gitignore by default
    # Also include here a potential exclude pattern or other options
    'grep_command': 'grep --exclude-dir=node_modules -RnP',
    'cpp': {
        'services': {
            'advertiser': {
                'regex': r'(\w*)\.advertiseService.*\(([^,]*),.*\)',
                'insert_code_after': '''
                    std::string resolvedName = {0}.resolveName({1});
                    ROS_ERROR_STREAM("ROS_SOURCE_GRAPH_PREFIX, SERVICE, ADVERTISER, {'file': " << {2} << " ,'line': " {3} << " ,'resolved_name': " << resolvedName << ",'name': "<< {1} << "}");
                    '''
            },
            'callers': {
                'regex': r'(\w*)\.serviceClient.*\(([^,]*).*\)',
                'insert_code_after': '''
                    std::string resolvedName = {0}.resolveName({1});
                    ROS_ERROR_STREAM("ROS_SOURCE_GRAPH_PREFIX, SERVICE, ADVERTISER, {'file': " << {2} << " ,'line': " {3} << " ,'resolved_name': " << resolvedName << ",'name': "<< {1} << "}");
                    '''
            },
        },
        'topics': {
            'advertiser': {
                # Regex explanation left to right
                # Match any word (valid var name) as capture group 1; find .advertise; then whatever
                # but no S (to exclude advertiseService and still catch the <msg> template); then (; then 2nd capture group whatever but no `,`;
                # then one `,`; then whatever; then );
                'regex': r'(\w*)\.advertise[^S]*\(([^,]*),.*\)',
                'insert_code_after': '''
                    std::string resolvedName = {0}.resolveName({1});
                    ROS_ERROR_STREAM("ROS_SOURCE_GRAPH_PREFIX, SERVICE, ADVERTISER, {'file': " << {2} << " ,'line': " {3} << " ,'resolved_name': " << resolvedName << ",'name': "<< {1} << "}");
                    '''
            },
            'subscribers': {
                'regex': r'(\w*)\.subscribe*\(([^,]*),.*\)',
                'insert_code_after': '''
                    std::string resolvedName = {0}.resolveName({1});
                    ROS_ERROR_STREAM("ROS_SOURCE_GRAPH_PREFIX, SERVICE, ADVERTISER, {'file': " << {2} << " ,'line': " {3} << " ,'resolved_name': " << resolvedName << ",'name': "<< {1} << "}");
                    '''
            },
        }
    },
    'python': {
        'services': {
            'advertiser': {
                'regex': r'rospy.Service\(([^,]*),.*\)',
                'insert_code_after': '''
                    resolved_name = rospy.resolved_name({0})
                    rospy.logerr("ROS_SOURCE_GRAPH_PREFIX File: {1},Line: {2},ResolvedName: {{resolved_name}},Name: {0}")
                '''
            },
            'callers': {
                'regex': r'rospy.ServiceProxy\(([^,]*),.*\)',
                'insert_code_after': '''
                    resolved_name = rospy.resolved_name({0})
                    rospy.logerr("ROS_SOURCE_GRAPH_PREFIX File: {1},Line: {2},ResolvedName: {{resolved_name}},Name: {0}")
                '''
            },
        },
        'topics': {
            'advertiser': {
                'regex': r'rospy.Publisher\(([^,]*),.*\)',
                'insert_code_after': '''
                    resolved_name = rospy.resolved_name({0})
                    rospy.logerr("ROS_SOURCE_GRAPH_PREFIX File: {1},Line: {2},ResolvedName: {{resolved_name}},Name: {0}")
                '''
            },
            'subscribers': {
                'regex': r'rospy.Subscriber\(([^,]*),.*\)',
                'insert_code_after': '''
                    resolved_name = rospy.resolved_name({0})
                    rospy.logerr("ROS_SOURCE_GRAPH_PREFIX File: {1},Line: {2},ResolvedName: {{resolved_name}},Name: {0}")
                '''
            }
        }
    }
}
