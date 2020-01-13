int32 regionId
nav_msgs/OccupancyGrid map
roborock_app/CleanPath[] paths
#虚拟墙
roborock_app/Line[] virtualWalls
#禁区
roborock_app/Rect[] forbiddens
#减速带
roborock_app/Rect[] speedBumps
#定位框
roborock_app/LocateRect[] locateRects
#机器人位置
geometry_msgs/Point robotPosition
#标记点
roborock_app/MarkPoint[] markPoints
#路径填充框
roborock_app/Rect[] pathFillRects
#激光数据 -- 单位:地图点，坐标按照x0,y0,x1,y1...排列
int32[] laserData
