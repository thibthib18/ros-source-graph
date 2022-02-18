import subprocess
from config import config

# for each language
languages = ['python', 'cpp']
# for each topic(sub+adv), srv (adv+call)
# find all matching file+line
# for each match
# append the lines

cmd = r'grep --exclude-dir=node_modules -RnP ".*(\w*)\.advertise[^S]*\(([^,]*),.*\)" | cut -d: -f1-2'


lang_config = config['python']

language = 'python'
resource = 'topics'
type = 'subscribers'


def get_match_command(config, language, resource, type):
    return fr'{config["grep_command"]} {config[language][resource][type]["regex"]} | cut -d: -f1-2'


def find_matching_file_lines(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    out = p.stdout.read()
    filesLines = out.decode('utf-8').split('\n')
    matches = []
    for fileLine in filesLines:
        file, line = fileLine.split(':')
        matches.append({'file': file, 'line': line})
