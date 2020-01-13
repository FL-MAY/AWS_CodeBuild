#值参见RobotStatCode.msg
uint8 stat
#值参见StatForAppCode.msg
uint8 stat_for_app

#如果stat是ERROR状态
int16 error_code

#如果stat是AUTOMATIC_CLEAN状态
#当前清扫的面积，单位平方厘米
int32 clean_area

#当前清扫时长，单位秒
int32 clean_duration

#剩余电量
int8 power

int32 total_clean_area
int32 total_clean_duration
int8 clean_water_level
int8 dirty_water_level

#值定义在UpDown.msg中
#刷盘状态
uint8 brush_disc_stat
#吸水扒状态
uint8 water_suck_stat

#以下字段值定义在Speed.msg中
uint8 brush_disc_speed
uint8 water_pump_speed
uint8 water_suck_speed

#行进速度
float32 walk_speed

#行进方向
#在Direction.msg中定义
uint8 walk_direction
