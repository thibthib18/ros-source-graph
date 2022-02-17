






**** Data collection *****

** Python
Method: wrap `rospy.Service` and `rospy.ServiceProxy` to log the service name when call/adv. Then run the system and parse the created log.

*** Services ******
**** Advertisers ****
add the following before end of `rospy.Service` constructor
```python
caller_file = inspect.stack()[1][1]
caller_line = inspect.stack()[1][2]
service_name = self.resolved_name
# somehow log this
```

**** Callers ****
add the following before end of `rospy.ServiceProxy` constructor
```python
caller_file = inspect.stack()[1][1]
caller_line = inspect.stack()[1][2]
service_name = self.resolved_name
# somehow log this
```
Note: `pylib.utils.utils.call_service` might need to be updated as well for more accuracy

*** Topics ******
Essentially the same method, but with `rospy.Publisher`, and `rospy.Subscriber`
Note: Maybe beneficial to include `rospy.wait_for_message` as well.

Note: logging might be done via `rospy.logerr`, with some prefix to help further parsing

** C++
Python method: Could the same method as python may be applied via macros? This will at least require recompiling some ROS modules afterwards and potentially include some extra libs.
--> might be complicated, worth investigating other solutions. After looking at it, seems possible but not easy.

Make some shell script or vim with recursive macro:
- find all the lines that contains e.g. `advertiseService`
- already get the line and file at this point
- extract first arg of `advertiseService` to get the service name
- get resolved_name via `NodeHandle::resolveName`.
- log it
---> sounds hacky af, but why not
Note: if that "direct code update" method works for C++, it might easily enough modifiable to use for Python, maybe more languages. Also might be more reproducible?

Algo could go as following:
```pseudo/python
line_identifier = "advertiseService("
filesLines = find_all_files_with(line_identifier) # array of ["path:line"]
for fileLine in filesLines:
  append_logger(fileLine)

def append_logger(fileLine):
  # those 2 lines for C++ only
  unresolved_name = extract_first_arg(fileLine)
  node_handle = extract_node_handle(fileLine) # that's the variable before .serviceClient
  append_lines("log(file,line,unresolved_name, name)")
```


Note: Logging via ROS might be a good idea again: no need to include more dependencies + also works for python.

Not that many C++ nodes, do it manually?
---> boring af, reproducibility very low.

** Skaarhoj
Skaarhoj interacts with ROS via rosbridge, and in C++.... Probably there are ways to handle, but I'm just not interested in this one for now.

** TS/JS
Our frontend is basically a single big ROS node through the rosbridge and the API/ROS calls are pretty well defined already. I dont see much benefit to supporting this as it would mainly always refer to files in the `api` directory, so I can do that just fine by myself.



**** Data extraction *****
Goal is creating a json in this format:
```json
[
  "/backend/myservice/set": {
    "advertiser": "/home/sv/main/backend/script.py:16",
    "callers": [
      "/home/sv/main/backend/client.py:22",
      "/home/sv/main/cinematography/otherClient.py:22"
    ]
  }
  "/backend/somservice/get": {
    "advertiser": "/home/sv/main/package1/script.py:16",
    "callers": [
      "/home/sv/main/backend/client.py:24",
      "/home/sv/main/cinematography/otherClient.py:44"
    ]
  }
]
```
This may then be loaded by the IDE to get and navigate the list of callers/subscribers.

*** Example integration usage workflow
Situation: cursor over a topic name in a Publisher, wondering who's listening to it
- press <leader>rtl  (like Ros Topic Listeners, or whatever)
- IDE looks at `topics.json`, get the listeners locations(file+line) via `topics[<yourtopic>][listeners]`
- IDE returns you a navigable list of all the listeners
- now inspect the next node and maybe re iterate :D

Note: cursor might be over a *non resolved* name. Maybe the data structure should give a way to
