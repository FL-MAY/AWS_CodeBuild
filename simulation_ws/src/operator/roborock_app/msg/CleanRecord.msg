# 清扫记录id,要求唯一
int32 id 
# 清扫子区域名称
string sub_region_name
# 清扫时刻
# the new time in UTC seconds from the epoch.
std_msgs/Time clean_time
# 清扫配置，参考ConfigIndex
uint8 clean_config
# 清扫面积，单位平米 
float32 clean_area
# 清扫时长，单位分钟
float32 clean_duration
# 用水量, 单位升
float32 water_used
# 耗电量, 瓦时?
float32 power_cost
# 清扫覆盖率，百分数
float32 cover_rate
