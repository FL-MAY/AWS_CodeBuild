# Type defines the mission type
# ID is a key in database for a more detailed mission description 
# Candidate Action type
uint8 AUTOMATIC     = 22  # Automatic mode
uint8 MANUAL        = 28  # Manual mode
uint8 MAPPING       = 29  # Generate map
uint8 TRACKING      = 30  # Manually draw a path
uint8 CHARGING      = 23  # Charging
uint8 UPDATING	    = 25  # Updating  	
#end Actions 
uint8 type
uint32 id
