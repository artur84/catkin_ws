## *********************************************************
## 
## File autogenerated for the controller package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'k1 gain', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'k1', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'k2 gain', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'k2', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'k3 gain', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'k3', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'k4 gain', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'k4', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Position of follower point, x coord', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'Pfm_x', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -10.0, 'type': 'double'}, {'srcline': 259, 'description': 'Position of follower point, y coord', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'Pfm_y', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -10.0, 'type': 'double'}, {'srcline': 259, 'description': 'Enable visual debug', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'enable_debug', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'The control laws to use', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'ctrl_laws', 'edit_method': "{'enum_description': 'An enum to set control_laws', 'enum': [{'srcline': 20, 'description': 'Parametric follower point', 'srcfile': '/home/arturo/catkin_ws/src/emotion/controller/cfg/Controller.cfg', 'cconsttype': 'const char * const', 'value': 'parametric', 'ctype': 'std::string', 'type': 'str', 'name': 'Parametric'}, {'srcline': 21, 'description': 'Follower point on y-axis', 'srcfile': '/home/arturo/catkin_ws/src/emotion/controller/cfg/Controller.cfg', 'cconsttype': 'const char * const', 'value': 'singular', 'ctype': 'std::string', 'type': 'str', 'name': 'Singular'}, {'srcline': 22, 'description': 'Follower point on y-axis plus position error', 'srcfile': '/home/arturo/catkin_ws/src/emotion/controller/cfg/Controller.cfg', 'cconsttype': 'const char * const', 'value': 'singular_pos', 'ctype': 'std::string', 'type': 'str', 'name': 'SingularPos'}, {'srcline': 23, 'description': 'Generalized velocity controller', 'srcfile': '/home/arturo/catkin_ws/src/emotion/controller/cfg/Controller.cfg', 'cconsttype': 'const char * const', 'value': 'velocity', 'ctype': 'std::string', 'type': 'str', 'name': 'Velocity'}]}", 'default': 'parametric', 'level': 0, 'min': '', 'type': 'str'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

controller_params_Parametric = 'parametric'
controller_params_Singular = 'singular'
controller_params_SingularPos = 'singular_pos'
controller_params_Velocity = 'velocity'
