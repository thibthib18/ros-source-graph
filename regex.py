import re

regex = r'\(\s*([^)]+?)\s*\)'
r = re.compile(regex)
line = 'getService = privateNodeHandle.advertiseService("get", &Shot::getShotCallback, this);'
#result = r.findall()
#args = result[0].split(',')


def print_match(m):
    if m is not None:
        print(m.group(1))
    else:
        print('No match')


line1 = 'a = nh.advertise<sv::msg>("get", arg2, arg3)'
line2 = 'a = nh.advertise("get", arg2, arg3)'
line3 = 'a = nh.advertiseService("get", arg2, arg3)'
reg = r'(\w*)\.advertise[^S]*\('
#reg = r'(\w*)\.adv.*\('
r = re.compile(reg)
m = r.search(line1)
print_match(m)
m = r.search(line2)
print_match(m)
m = r.search(line3)
print_match(m)
