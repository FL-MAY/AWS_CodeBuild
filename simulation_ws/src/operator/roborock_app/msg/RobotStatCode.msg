#其他机器人状态
uint8 IDLE = 20						#待命
uint8 PREPARING = 21				#准备中，定位失败，无地图等都会留在这个状态
uint8 AUTOMATIC = 22				#自动任务就绪， 可执行GOTO任务或自动清洁任务
uint8 AUTOMATIC_CLEAN = 23			#自动清洁任务中
uint8 AUTOMATIC_GOTO = 24			#GOTO中
uint8 MANUAL = 25					#手动模式
uint8 MAPPING = 26					#建图模式
uint8 TRACKING = 27					#示教模式            
uint8 CHARGING = 28					#充电中
uint8 ERROR = 29					#错误
uint8 UPDATING = 30					#升级中
uint8 PAUSE = 31					# 暂停中 
uint8 EMG_STOP = 32					#急停拍下
#other stats ..
