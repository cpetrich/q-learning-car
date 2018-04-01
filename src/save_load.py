import json

# Author: Chris Petrich
# Date: Apr 2018

# massively reduced version of the original version
#   maintaining original interface of save()

def _to_json(dct):
    """convert keys to str, py2 and py3"""
    if isinstance(dct, dict):
        return {str(key).replace(' ',''):dct[key] for key in dct}
    return dct

def _str2inttuple(s):
    """parse string as tuple of integers"""
    return tuple([int(v) for v in s.split('(',1)[1].split(')',1)[0].split(',')])

def _from_json(dct):
    """convert string keys back to tuples"""
    if not isinstance(dct, dict): return dct
    for key in dct:
        break
    if isinstance(key, str) and key.startswith('('):
        return {_str2inttuple(key): dct[key] for key in dct}
    return dct

def save(filename, head_module, data_dict):
    """Outputs only data_dict"""
    # NB: zipping this up could reduce space to 30%.
    # remove .py and add .json
    filename = filename[:-3] if filename.lower().endswith('.py') else filename
    filename = filename if filename.lower().endswith('.json') else filename+'.json'
    with open(filename,'w') as f:
        json.dump({outer_key:_to_json(data_dict[outer_key]) for outer_key in data_dict}, f, separators=(',', ':'))

def load(filename):
    # remove .py and add .json
    filename = filename[:-3] if filename.lower().endswith('.py') else filename
    filename = filename if filename.lower().endswith('.json') else filename+'.json'
    with open(filename) as f:
        data=json.load(f)
    return {'data':{key:_from_json(data[key]) for key in data}}
