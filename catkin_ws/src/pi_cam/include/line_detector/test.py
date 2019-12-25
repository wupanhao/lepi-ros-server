#!coding:utf-8
import yaml
import io
import json
if __name__ == '__main__':
	stream = io.open('default.yaml', 'r',encoding='utf-8')
	calib_data = yaml.safe_load(stream)
	print(json.dumps(calib_data, encoding='utf-8', ensure_ascii=False))
	# print(calib_data['颜色'.decode('utf-8')]['黄色'.decode('utf-8')]['min'.decode('utf-8')])
	with io.open("./test.yaml", "w",encoding = 'utf-8') as yaml_file:
		yaml_obj = {}
		yaml_obj["白色".decode('utf-8')] = '三四五'.decode('utf-8')
		yaml.dump(yaml_obj, yaml_file,allow_unicode = True)