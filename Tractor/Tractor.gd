@tool
extends VehicleBody3D

@export var targetSpeed:float = 0
@export var accelerationDecelerationForce:float = 10000
@export var hideObstacles:bool:
	set(hideFlag):
		if (hideObstacles != hideFlag):
			updateObstacleVisibility(hideFlag)
		hideObstacles = hideFlag
	get:
		return hideObstacles

@export var hideEverythingButWheels:bool:
	set(hideFlag):
		if (hideEverythingButWheels != hideFlag):
			updateEverythingButWheelsVisibility(hideFlag)
		hideEverythingButWheels = hideFlag
	get:
		return hideEverythingButWheels

@export var implementWidth:float = 10:
	set(newWidth):
		if (implementWidth != newWidth):
			updateImplementWidth(newWidth)
		implementWidth = newWidth
	get:
		return implementWidth

@export var implementColliding:bool = true:
	set(colliding):
		if (implementColliding != colliding):
			$CollisionShape_Implement.disabled = !colliding
		implementColliding = colliding
	get:
		return implementColliding

@export var implementVisible:bool = true:
	set(newVisible):
		if (implementVisible != newVisible):
			$CollisionShape_Implement.visible = newVisible
			$Implement.visible = newVisible
		implementVisible = newVisible
	get:
		return implementVisible

@export var crosshairVisible:bool = true:
	set(newVisible):
		if (crosshairVisible != newVisible):
			$Crosshair.visible = newVisible
		crosshairVisible = newVisible
	get:
		return crosshairVisible

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
	engine_force = -accelerationDecelerationForce * max(min(speedError * 10, 1), -1) 

	# Calculate location/orientation
	# This simulation uses the same system as dual-antennas,
	# and the origin of the antenna itself is used here
	# (Although there is no need for second antenna here, the second
	# one is just expected to be on the left side of the one present)
	var globalOrigin = $GNSSAntenna.global_transform.origin

	# Shifting coords 1 km north so we can keep calculations simpler
	# (Staying north and east all the time)
	latitude = (-globalOrigin.z + 1000) / (earthsCircumference / 4) * 90
	longitude = (globalOrigin.x + 1000) / (earthsCircumference / 4) * 90
	altitude = globalOrigin.y
	
	heading = rad_to_deg(atan2(self.global_transform.basis.z.x, -self.global_transform.basis.z.z))
	
#	while (heading < 0):
#		heading += 360
	
	roll = rad_to_deg(asin(self.global_transform.basis.x.y))

	$SteeringWheelBase/Rotator.rotation = Vector3(0, 0, deg_to_rad(rad_to_deg(steering) / 45 * 450))

	# Godot's VehicleBody3D is quite buggy
	# (like they say and admit even on the documentation, so it's known).

	# First I tried to make the front axle as they are in real tractors
	# (hinge in the center). This failed first spectacularly by throwing the
	# tractor to the sky. After a lot of tuning this experiment ended into front
	# wheels burying into ground like it was quicksand. Also tractor kept
	# colliding into "ghost" obstacles everywhere, so it didn't work out.

	# Then adding objects as childs of VehicleWheel3Ds led them to be somehow
	# "mirrored" (like normals were inverted) in the game but not in editor.
	# So now copying just the "essential" parameters from the VehicleWheel3Ds
	# to the visual objects. Back wheels are kept at their original "heights"
	# to behave more like in a real tractor.
	# Front wheels' suspension are allowed to move more to mimic a hinged
	# front axle.

	# TODO: Try Jolt or some of the "3rd party" vehicle implementations...
	# (just now too lazy to try)

	if (!Engine.is_editor_hint()):
		# And running this in editor (@tool script) turns the wheels
		# wrong way (like they are meant to be driven backwards)
		# so not copying the rotation in editor.
		$RearWheelObject_Left.rotation = $VehicleWheel_RearLeft.rotation
		$RearWheelObject_Right.rotation = $VehicleWheel_RearRight.rotation
		$FrontWheelObject_Left.rotation = $VehicleWheel_FrontLeft.rotation
		$FrontWheelObject_Left.transform.origin.y = $VehicleWheel_FrontLeft.transform.origin.y
		$FrontWheelObject_Right.rotation = $VehicleWheel_FrontRight.rotation
		$FrontWheelObject_Right.transform.origin.y = $VehicleWheel_FrontRight.transform.origin.y

func updateObstacleVisibility(hideFlag):
	var modelRootNode = $Tractor_Wheeless/Sketchfab_model/eeff7b40aecd4dd1a8f4adcae6c5bdec_fbx/RootNode/AM115_026
	
	modelRootNode.get_node("ExhaustPipe").visible = !hideFlag
	modelRootNode.get_node("Glass").visible = !hideFlag
	modelRootNode.get_node("Left_Pillars").visible = !hideFlag
	modelRootNode.get_node("LMirror_1").visible = !hideFlag
	modelRootNode.get_node("LMirror_2").visible = !hideFlag
	modelRootNode.get_node("LMirror_3").visible = !hideFlag
	modelRootNode.get_node("LMirror_4").visible = !hideFlag
	modelRootNode.get_node("LMirror_5").visible = !hideFlag
	modelRootNode.get_node("Mirror_ReflectiveSurface").visible = !hideFlag
	modelRootNode.get_node("Muffler").visible = !hideFlag
	modelRootNode.get_node("Right_Pillars").visible = !hideFlag
	modelRootNode.get_node("RMirror_1").visible = !hideFlag
	modelRootNode.get_node("RMirror_2").visible = !hideFlag
	modelRootNode.get_node("RMirror_3").visible = !hideFlag
	modelRootNode.get_node("RMirror_4").visible = !hideFlag
	modelRootNode.get_node("RMirror_5").visible = !hideFlag
	
func updateEverythingButWheelsVisibility(hideFlag):
	$Tractor_Wheeless.visible = !hideFlag
	$FakeFrontAxle.visible = !hideFlag
	$GNSSAntenna.visible = !hideFlag

func updateImplementWidth(newWidth:float):
	$Implement.mesh.size.x = newWidth
	$CollisionShape_Implement.shape.size.x = newWidth
