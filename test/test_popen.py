import os
import re
data = ''
bin_dir = os.path.expanduser(
    '~')+'/Lepi_Data/ros/smart_audio_node'
cmd = 'bash -c "cd %s && LD_LIBRARY_PATH=./  ./asr_offline %s "' % (
    bin_dir, data)
result = os.popen(cmd)
res = result.read()
# print(res)
array = re.findall(
    r"Result: \[ confidence=(\d+) grammar=(\d+) input=(.+) \]", res, re.S)
# print(array)
