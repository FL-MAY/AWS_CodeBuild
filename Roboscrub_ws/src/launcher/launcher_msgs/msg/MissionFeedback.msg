# State: 	Robot current state
# Error code:   Current error code
#		0 means OK
# id:           Current Mission ID
# Clean Config: Current clean config     
# Candidate States
uint8 IDLE 	= 20
uint8 PREPARING = 21
uint8 AUTOMATIC = 22
uint8 CHARGING  = 23
uint8 ERROR 	= 24
uint8 UPDATING 	= 25
uint8 PAUSE 	= 26
uint8 EMG_STOP 	= 27
uint8 MANUAL 	= 28
uint8 MAPPING 	= 29
uint8 TRACKING 	= 30
# end States
uint8  state
uint32 id       
uint8 error_code
uint8 clean_config
