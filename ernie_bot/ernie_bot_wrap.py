# -*- coding: utf-8 -*-
import erniebot, json
from jsonschema import validate


class PromptJson:
	def __init__(self, rulers) -> None:
		self.rulers_str = '请根据下面的scheame描述生成给定格式json,只返回json数据,不要其他内容。'
		self.scheame_str = ''
		self.example_str = ''

		self.set_rulers(rulers)
		self.set_scheame(self.json_obj())
		self.set_example(self.example())

	def json_obj(self):
		return '''```{'type':'string'}```'''

	def example(self):
		return '正确的示例如下：'
	def __call__(self, *args, **kwargs):
		pass
    
	def set_scheame(self, json_obj):
		# json转字符串去空格,换行，制表符
		json_str = str(json_obj).replace(' ', '').replace('\n', '').replace('\t', '')
		# 加上三个引号
		json_str = '```' + json_str + '```'
		self.scheame_str = json_str

	def set_example(self, example_str:str):
		# 去空格,换行，制表符
		example_str = example_str.replace(' ', '').replace('\n', '').replace('\t', '')
		self.example_str = example_str

	def set_rulers(self, rulers):
		self.rulers_str = rulers.replace(' ', '').replace('\n', '').replace('\t', '')

	def __str__(self) -> str:
		return self.__repr__()
	
	def __repr__(self) -> str:
		return self.rulers_str + self.scheame_str + self.example_str
	
class ActionPrompt(PromptJson):
	def __init__(self) -> None:
		
		rulers = '''你是一个机器人动作规划者，需要把我的话翻译成机器人动作规划并生成对应的json结果，机器人工作空间参考右手坐标系。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
		# self.set_rulers(rulers)
		# self.set_scheame(self.json_obj())
		# self.set_example(self.example())

	def json_obj(self)->dict:
		schema_move = {'type': 'object', 'required': ['func', 'x', 'y', 'angle'],
               'porperties':{
                                'func': {'description': '移动', 'const': 'move'},
                                'x': {'description': 'x坐标, 前后移动, 向前移动正值，向后移动负值', 'type': 'number'},
                                'y': {'description': 'y坐标, 左右移动, 向左移动正值，向右移动负值', 'type': 'number'}, 
                                'angle': {'description': '旋转或者转弯角度，右转顺时针负值，左转逆时针正值', 'type': 'number'}
                            }
            }
		schema_beep = { 'type': 'object', 'required': ['func', 'time_dur','count','interval'],
				'properties': {'func': {'description': '蜂鸣器,需要发声时', 'const': 'beep'}, 
                   'time_dur': {'description': '蜂鸣器发声持续时间,,默认为0.2秒', 'type': 'number'},
				   'count': {'description': '发声次数,默认为1次', 'type': 'number'},
				   	'interval': {'description': '间隔时间,默认为0.2秒', 'type': 'number'},
				   }
		}

		schema_light = { 'type': 'object', 'required':['func', 'time_dur','count','interval'],
						'properties': {'func': {'description': '发光,需要照明时', 'const': 'light'}, 
						'time_dur': {'description': '照亮持续时间,默认为0.2秒', 'type': 'number'}},
						'count': {'description': '照亮次数,默认为1次', 'type': 'number'},
				   		'interval': {'description': '间隔时间,默认为0.3秒', 'type': 'number'},
		}
		schema_wait = { 'type': 'object', 'required':['func', 'time_dur'],
						'properties': {'func': {'description': '需要等待时', 'const': 'wait'}, 
						'time_dur': {'description': '需要停止等待时间,默认为0.2秒', 'type': 'number'}},

		}
		schema_actions = {'type': 'array', 'required': ['items'],
                  'items': {'anyOf': [schema_move, schema_beep, schema_light,schema_wait],
                        'minItems': 1
                    }
		}
		return schema_actions
	
	def example(self)->str:
		example = '''正确的示例如下：
					向左移0.1m, 向左转弯85度: ```[{'func': 'move', 'x': 0, 'y': 0.1, 'angle': 85}]```,
					向右移0.2m, 向前0.1m。 ```[{'func': 'move', 'x': 0.1, 'y': -0.2, 'angle': 0}]```,
					向右转弯85度, 向右移0.1m,。 ```[{'func': 'move', 'x': 0, 'y': -0.1, 'angle': -85}]```,
					蜂鸣器发声5秒。 ```[{'func': 'beep', 'time_dur': 5, 'count': 1, 'interval': 0.2}]```,
					蜂鸣器叫三次,间隔一秒。```[{'func': 'beep', 'time_dur': 0.2, 'count': 3, 'interval': 1}]```,
					发光5秒。 ```[{'func': 'light', 'time_dur': 5, 'count': 1, 'interval': 0.5}]```,
					发光5秒,然后等待三秒。 ```[{'func': 'light', 'time_dur': 5, 'count': 1, 'interval': 0.5},{'func': 'wait', 'time_dur': 3}]```,
					向右移0.2m, 向前0.1m。然后蜂鸣器发声一秒。```[{'func': 'move', 'x': 0.1, 'y': -0.2, 'angle': 0}, {'func': 'beep', 'time_dur': 1}]```,
					蜂鸣器叫三次，每次持续三秒,灯闪烁三次，之后原地旋转90度，向前0.1m。```[{'func': 'beep', 'time_dur': 3, 'count': 3, 'interval': 0.2}, {'func': 'light', 'time_dur': 0.2, 'count': 3, 'interval': 0.3}, {'func': 'move', 'x': 0, 'y': 0, 'angle': 90}, {'func': 'move', 'x': 0.1, 'y': 0, 'angle': 0}]```。
				'''
		return example
	
class HumAttrPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个人特征总结程序，需要根据描述把人的特征生成对应的json结果，如果有对应的描述就写入对应位置。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)

	def json_obj(self)->dict:
		'''
		0 = Hat - 帽子:0无1有
		1 = Glasses - 眼镜:0无1有
		2 = ShortSleeve - 短袖
		3 = LongSleeve - 长袖
		4 = UpperStride - 有条纹
		5 = UpperLogo - 印有logo/图案
		6 = UpperPlaid - 撞色衣服(多种颜色)
		7 = UpperSplice - 格子衫
		8 = LowerStripe - 有条纹
		9 = LowerPattern - 印有图像
		10 = LongCoat - 长款大衣
		11 = Trousers - 长裤
		12 = Shorts - 短裤
		13 = Skirt&Dress - 裙子/连衣裙
		14 = boots - 鞋子
		15 = HandBag - 手提包
		16 = ShoulderBag - 单肩包
		17 = Backpack - 背包
		18 = HoldObjectsInFront - 手持物品
		19 = AgeOver60 - 大于60
		20 = Age18-60 - =18~60
		21 = AgeLess18 - 小于18
		22 = Female - 0:男性; 1:女性
		23 = Front - 人体朝前
		24 = Side - 人体朝侧
		25 = Back - 人体朝后
		'''
		schema_attr = {'type': 'object', 
                'properties':{
                    'hat':{'type': 'boolean', 'description': '戴帽子真,没戴帽子假'},
					'glasses': {"type": 'boolean', 'description': '戴眼镜真,没戴眼镜假', 'threshold':0.15},
					'sleeve':{'enum': ['Short', 'Long'], 'description': '衣袖长短'},
					# 'UpperStride', 'UpperLogo', 'UpperPlaid', 'UpperSplice'	有条纹		印有logo/图案	撞色衣服(多种颜色) 格子衫
					'color_upper':{'enum':['Stride', 'Logo', 'Plaid', 'Splice'], 'description': '上衣衣服颜色'},
					# 'LowerStripe', 'LowerPattern'		有条纹		印有图像
					'color_lower':{'enum':['Stripe', 'Pattern'], 'description': '下衣衣服长短'},
					# 'LongCoat', 长款大衣
					'clothes_upper':{'enum':['LongCoat'], 'description': '上衣衣服类型', 'threshold':0.8},
					# 'Trousers', 'Shorts', 'Skirt&Dress'  长裤		短裤 	裙子/连衣裙
					'clothes_lower':{'enum':['Trousers', 'Shorts', 'Skirt_dress'], 'description': '下衣衣服类型'},
					'boots':{'type': 'boolean', 'description': '穿着鞋子真,没穿鞋子假'},
					'bag':{'enum': ['HandBag', 'ShoulderBag', 'Backpack'], 'description': '带着包的类型'},
					'holding':{'type': 'boolean', 'description': '持有物品为真', 'threshold':0.5},
					'age':{'enum': ['Old', 'Middle', 'Young'], 'description': '年龄,小于18岁为young, 18到60为middle, 大于60为old'},
					'sex':{'enum': ['Female', 'Male'], 'threshold':0.6},
					'direction':{'enum': ['Front', 'Side', 'Back'], 'description': '人体朝向'},
					},
                "additionalProperties": False
            }
		return schema_attr
	
	def example(self)->str:
		example = '''正确的示例如下：
					一个带着眼镜的老人: ```{'glasses': True, 'age': 'old'}```,
					一个带着帽子的中年人: ```{'hat': True, 'age': 'middle'}``` ,
					穿着短袖的带着眼镜的人: ```{'glasses': True, 'clothes': 'short'}``` 。
				'''
		return example
	
class ErnieBotWrap():

	def __init__(self):
		erniebot.api_type = 'aistudio'
		erniebot.access_token = '1dcb78a336d3164564a89369c30591f8c8e9e79d'
		self.msgs = []
		# self.model = 'ernie-4.0'
		self.model = 'ernie-3.5'
		# self.model = 'ernie-turbo'
		# self.model = "ernie-text-embedding"
		# self.model = "ernie-vilg-v2"
		self.prompt_str = '请根据下面的描述生成给定格式json'

	@staticmethod
	def get_mes(role, dilog):
		data = {}
		if role == 0:
			data['role'] = 'user'
		elif role ==1:
			data['role'] = 'assistant'
		data['content'] = dilog	
		return data

	def set_promt(self, prompt_str):
		# str_input = prompt_str
		# self.msgs.append(self.get_mes(0, str_input))
		# response = erniebot.ChatCompletion.create(model=self.model, messages=self.msgs, system=prompt_str)
		# str_res = response.get_result()
		# self.msgs.append(self.get_mes(1, str_res))
		# print(str_res)
		# print("设置成功")
		self.prompt_str = prompt_str
		# print(self.prompt_str)


	def get_res(self, str_input, record=False, request_timeout=5):
		if len(str_input)<1:
			return False, None
		start_str = " ```"
		end_str = " ```, 根据这段描述生成给定格式json"
		str_input = start_str + str_input + end_str
		msg_tmp = self.get_mes(0, str_input)
		
        # record 表示是否进行多轮对话
        # msgs格式(list元素要为单数)示例如下：
		'''
		[
			{
				"role": "user",
				"content": "你好啊"
			},
			{
				"role": "assistant",
				"content": "你好，我是文心一言"
			},
			{
				"role": "user",
				"content": "深圳周末去哪里玩好?"
			}
		]
		
		'''

		if record:
			self.msgs.append(msg_tmp)
			msgs = self.msgs
		else:
			msgs = [msg_tmp]
		# Create a chat completion
        # top_p 控制输出文本的多样性，越小输出的文本越单一
        # system 角色扮演   github看     system: A message that tells the LLM how to interpret the conversations.
		# Create a chat completion
		try:
			response = erniebot.ChatCompletion.create(model=self.model, messages=msgs, system=self.prompt_str, top_p=0.1,
											_config_=dict(api_type="AISTUDIO",), request_timeout=request_timeout)
		except Exception as e:
			# print(e)
			return False, None
		# _config_=dict(api_type="QIANFAN",)
		# _config_=dict(api_type="AISTUDIO",)
		# print(response)
		str_res = response.get_result()
		if record:
			self.msgs.append(self.get_mes(1, str_res))
		return True, str_res
	
	@staticmethod
	def get_json_str(json_str:str):
		try:
			index_s = json_str.find("```json") + 7
		except Exception as e:
			index_s = 0
		try:
			index_e = json_str[index_s:].find("```") + index_s
		except Exception as e:
			index_e = len(json_str)
		import json
		msg_json = json.loads(json_str[index_s:index_e])
		return msg_json
	
	def get_res_json(self, str_input, record=False, request_timeout=15):
		state, str_res = self.get_res(str_input, record, request_timeout)
		if state:
			obj_json = self.get_json_str(str_res)
			return obj_json
		else:
			return None

if __name__ == "__main__":
	ernie = ErnieBotWrap()
	# 设置prompt
	# ernie.set_promt(str(ActionPrompt()))
	ernie.set_promt(str(ActionPrompt()))
	while True:
		print("用户")
		str_tmp = input("输入:")
		if len(str_tmp)<1:
			continue
		# Create a chat completion
		print("文心一言")
		# _, str_res = ernie.get_res(str_tmp)
		json_res = ernie.get_res_json(str_tmp)
		print("输出:",json_res)
