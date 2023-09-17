extends VehicleBody3D

@export var targetSpeed:float = 0
@export var accelerationDecelerationForce:float = 10000

# Latitude and longitude are GNSS-antenna's values
var latitude:float = 0		# in deg
var longitude:float = 0		# in deg

var altitude:float = 0		# in m
var heading:float = 0		# in deg
var roll:float = 0			# in deg
var measuredSpeed:float = 0	# in km/h

var lastPos:Vector3

const earthsCircumference = 40075.017e3	# in m

func _physics_process(delta):
	# Simplest ever speed controller...
	var posChange = self.global_transform.origin - lastPos
	lastPos = self.global_transform.origin
	
	# Calculate speed along tractor's Z-axis (in km/h):
	measuredSpeed = (posChange.dot(self.global_transform.basis.z) / delta) * 3.6 

	var speedError = measuredSpeed - targetSpeed

	# Set engine_force to just scaled speed error with limits
	# (works surprisingly well being this simple)
	self.engine_force = -accelerationDecelerationForce * max(min(speedError * 10, 1), -1) 

	# Calculate location/orientation
	# In this simulation I used the dimensions from pre-existing
	# tractor with dual-antennas, so the origin of the antenna itself is used here
	# (Although there is no need for second antenna here)
	var globalOrigin = $GNSSAntenna.global_transform.origin

	# Shifting coords 1 km north so we can keep calculations simpler
	# (Staying north and east all the time)
	latitude = (-globalOrigin.z + 1000) / (earthsCircumference / 4) * 90
	longitude = (globalOrigin.x + 1000) / (earthsCircumference / 4) * 90
	altitude = globalOrigin.y
	
	heading = rad_to_deg(atan2(self.global_transform.basis.z.x, -self.global_transform.basis.z.z))
	roll = rad_to_deg(asin(self.global_transform.basis.x.y))
