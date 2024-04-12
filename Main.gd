extends Node3D

@onready var terrain = $Terrain
@onready var terrainCollisionShape = $Terrain/StaticBody3D/CollisionShape3D

var udpServer:UDPServer
var clientPeer:PacketPeerUDP
var peers = []

const udpServerPort:int = 8888

const udpDestAddress:String = "192.168.5.10"
const udpDestPort:int = 9999

class SteerSettings:
	var kp:int = 0
	#var lowPWM:int = 10 # band of no action (calculation moved to func calcSteeringPID)
	#int16_t wasOffset = 0;
	var minPWM:int = 0;
	var highPWM:int = 0;		# max PWM value
	#float steerSensorCounts = 30;
	#float AckermanFix = 1;        // sent as percent

var steerSettings_Received:SteerSettings = SteerSettings.new()
var steerSettings_ReceivedUpTime_ms:float = 0

var maxDrivingSpeed:float = 24

# Called when the node enters the scene tree for the first time.
func _ready():
# This does not work here (handled in _process instead):
#	updateTerrainCollisionShape()
	
	loadConfig()

	clientPeer = PacketPeerUDP.new()
	clientPeer.set_broadcast_enabled(true)
	clientPeer.set_dest_address(udpDestAddress, udpDestPort)

	udpServer = UDPServer.new()
	
	udpServer.listen(udpServerPort)

func _exit_tree():
	if (ffbInitSuccessful):
		destroyForceFeedbackEffect()
	saveConfig()

func updateTerrainCollisionShape():
	var terrainMaterial:ShaderMaterial = terrain.material_override
	var heightmapTexture:Texture2D = terrainMaterial.get_shader_parameter("heightmap")
	var heightMultiplier:float = terrainMaterial.get_shader_parameter("heightMultiplier")
	var edgeRise:float = terrainMaterial.get_shader_parameter("edgeRise")
	var heightMapImage = heightmapTexture.get_image()
	heightMapImage.convert(Image.FORMAT_RF)
	# Resize could be used to make CollisionShape lighter?
	var data = heightMapImage.get_data().to_float32_array()
	for i in range(0, data.size()):
		data[i] *= heightMultiplier
#		data[i] = randf_range(-1000, 1000)

	var shape:HeightMapShape3D = HeightMapShape3D.new()
	shape.map_width = heightMapImage.get_width()
	shape.map_depth = heightMapImage.get_height()
	
	for i in range (heightMapImage.get_width()):
		# North / south walls
		data[i] += edgeRise
		data[i + ((heightMapImage.get_width() - 1) * (heightMapImage.get_height()) - 1)] += edgeRise
	for i in range(1, (heightMapImage.get_height()) - 1):
		# West / east walls
		data[i * heightMapImage.get_width()] += edgeRise
		data[i * heightMapImage.get_width() + heightMapImage.get_width() - 1] += edgeRise
		
#	for i in range(0, heightMapImage.get_height(), 2):
#		for ii in range (0, heightMapImage.get_width()):
#			data[i * heightMapImage.get_width() + ii] += edgeRise
	
	shape.map_data = data
	terrainCollisionShape.shape = shape
	var scaleVal:float = 1000.0/float(heightMapImage.get_width())
	terrainCollisionShape.scale = Vector3(scaleVal, 1, scaleVal)

var terrainCollisionShapeUpdated:bool = false

var keyboardSpeedChangeTimer:float = 0
var keyboardAngleChangeTimer:float = 0

var joystickSpeedAcceleration:float = 10
var joystickSpeedDeceleration:float = 30
var joystickSpeedSetpoint:float = 0

var steerSwitchPressed:bool = false
var steeringWheelPosition:float = 0		# -1...1
var steerAngleSetpoint:float = 0

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	if (!terrainCollisionShapeUpdated):
		# This is very dirty way to make sure the shadermatererial's
		# get_shader_parameter-function gives sensible values before
		# using them in updateTerrainCollisionShape
		# (updateTerrainCollisionShape does not work when called from _ready)
		updateTerrainCollisionShape()
		terrainCollisionShapeUpdated = true
	
	if (Input.is_action_just_pressed("full_screen_toggle")):
		if (DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN):
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
		else:
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)

	cameraSwitch()
	
	$Panel_JoystickFFBSettings.visible = $Panel_Controls/CheckBox_ShowJoystickSettings.button_pressed
	$Panel_SteerSettings.visible = $Panel_Controls/CheckBox_ShowJoystickSettings.button_pressed && $Panel_JoystickFFBSettings/CheckBox_ShowSteerSettings.button_pressed
	$Tractor.hideObstacles = $Panel_Controls/CheckBox_HideObstacles.button_pressed
	
	if (steerSettings_ReceivedUpTime_ms != 0):
		var timeElapsed:float = (Time.get_ticks_msec() - steerSettings_ReceivedUpTime_ms) * 0.001
		$Panel_SteerSettings/Label_TimeSinceParamsReceived.text = "Time since received: %1.1f s" % timeElapsed
	else:
		$Panel_SteerSettings/Label_TimeSinceParamsReceived.text = "Time since received: not received"

const camera_InterpolationTime_Fast = 500
const camera_InterpolationTime_Slow = 2000
var camera_Interpolating:bool = false
var camera_InterpolationFraction:float = 0
var camera_InterpolationTime:int = 2000
var camera_InterpolateFrom:Camera3D
var camera_InterpolateTo:Camera3D
var camera_InterpolationStartTime:int = 0

func cameraSwitch():
	var newCamera:Camera3D = null
	var uptime:int = Time.get_ticks_msec()

	# Default fast interpolation
	camera_InterpolationTime = camera_InterpolationTime_Fast
	
	if (Input.is_action_just_pressed("camera_flyer")):
		$FirstPersonFlyer.set_LocationOrientation(self.transform.affine_inverse() * get_viewport().get_camera_3d().get_global_transform())
		newCamera = $FirstPersonFlyer/Head/FirstPersonCamera
	elif (Input.is_action_just_pressed("camera_tractor_driver")):
		newCamera = $Tractor/Camera_Driver
	elif (Input.is_action_just_pressed("camera_tractor_follow")):
		newCamera = $Camera_Follow
	elif (Input.is_action_just_pressed("camera_tractor_angled")):
		newCamera = $Tractor/Camera_Angled
	elif (Input.is_action_just_pressed("camera_tractor_angled_2")):
		newCamera = $Tractor/Camera_Angled_2
	elif (Input.is_action_just_pressed("camera_tractor_back_fixed")):
		newCamera = $Tractor/Camera_Back_Fixed
	if false:	
		if Input.is_action_just_pressed("camera_memory_store"):
			var memCamera = get_node("MemoryCamera")
			var sourceCamera = get_viewport().get_camera()
			var sourceCameraGlobal = get_viewport().get_camera().get_global_transform()
			memCamera.global_transform = sourceCameraGlobal
			memCamera.near = sourceCamera.near
			memCamera.far = sourceCamera.far
			memCamera.fov = sourceCamera.fov
		if Input.is_action_just_pressed("camera_memory_recall"):
			newCamera = get_node("MemoryCamera")
			camera_InterpolationTime = camera_InterpolationTime_Slow

	if (newCamera):
		var oldCamera:Camera3D = get_viewport().get_camera_3d()
		if (newCamera == oldCamera):
			# Nothing to do really
			newCamera.current = true
			camera_Interpolating = false
		elif (camera_Interpolating):
			# Quick change on double press
			newCamera.current = true
			camera_Interpolating = false
		else:
			# Otherwise start interpolating
			camera_InterpolateFrom = oldCamera
			camera_InterpolateTo = newCamera
			camera_InterpolationStartTime = uptime
			camera_Interpolating = true
	
	if (camera_Interpolating):
		var currentFraction:float = smoothstep(camera_InterpolationStartTime, camera_InterpolationStartTime + camera_InterpolationTime, uptime)
		if (currentFraction >= 1):
			camera_InterpolateTo.current = true
			camera_Interpolating = false
		else:
			var currentPosition:Vector3 = camera_InterpolateFrom.global_transform.origin.lerp(camera_InterpolateTo.global_transform.origin, currentFraction)
			# Use quaternion slerp for smooth interpolation of rotation
			var currentOrientation:Quaternion = camera_InterpolateFrom.global_transform.basis.slerp(camera_InterpolateTo.global_transform.basis, currentFraction)
			var currentNear = camera_InterpolateFrom.near + (camera_InterpolateTo.near - camera_InterpolateFrom.near) * currentFraction
			var currentFar = camera_InterpolateFrom.far + (camera_InterpolateTo.far - camera_InterpolateFrom.far) * currentFraction
			var currentFov = camera_InterpolateFrom.fov + (camera_InterpolateTo.fov - camera_InterpolateFrom.fov) * currentFraction
			$CameraSwitchCamera.global_transform = Transform3D(currentOrientation, currentPosition)
			$CameraSwitchCamera.near = currentNear
			$CameraSwitchCamera.far = currentFar
			$CameraSwitchCamera.fov = currentFov
			$CameraSwitchCamera.current = true

var filteredSpeed:float = 0
const speedFilterCoeff:float = 0.05

func updateLocationOrientation(delta:float):
	$Panel_LocationOrientation/Label_Latitude_Value.text = "%1.7f N" % $Tractor.latitude
	$Panel_LocationOrientation/Label_Longitude_Value.text = "%1.7f E" % $Tractor.longitude
	$Panel_LocationOrientation/Label_Altitude_Value.text = "%1.2f m" % $Tractor.altitude
	$Panel_LocationOrientation/Label_Heading_Value.text = "%1.1f" % $Tractor.heading
	$Panel_LocationOrientation/Label_Roll_Value.text = "%1.1f" % $Tractor.roll
	var speedCoeff:float = pow(speedFilterCoeff, delta)
	filteredSpeed = $Tractor.measuredSpeed * (1.0 - speedCoeff) + filteredSpeed * speedCoeff
	$Panel_LocationOrientation/Label_Speed_Value.text = "%1.1f" % filteredSpeed

func _on_h_slider_steer_angle_value_changed(value):
	$Panel_Controls/Label_SteerAngle_Value.text = "%1.2f" % value

func _on_h_slider_target_speed_value_changed(value):
	$Panel_Controls/Label_TargetSpeed_Value.text = "%1.1f" % value

func _on_button_reset_steer_angle_and_speed_pressed():
	$Panel_Controls/HSlider_SteerAngle.value = 0
	$Panel_Controls/HSlider_TargetSpeed.value = 0
	joystickSpeedSetpoint = 0

var timeAfterPAOGIMessage:float = 0
var timeAfterValidSteerAngleSetpoint:float = 0

func _physics_process(delta):
	if (Input.is_action_just_pressed("reset_angle_and_speed")):
		_on_button_reset_steer_angle_and_speed_pressed()

	if (Input.is_action_just_pressed("steer_switch")):
		steerSwitchPressed = !steerSwitchPressed

	keyboardSpeedChangeTimer += delta

	if ($Panel_JoystickFFBSettings/CheckBox_JoystickSpeed.button_pressed):
		var speedChange:float = (Input.get_action_strength("Joystick_accelerate") * joystickSpeedAcceleration - 
				Input.get_action_strength("joystick_decelerate") * joystickSpeedDeceleration) * delta
		
		if ($Panel_JoystickFFBSettings/CheckBox_JoystickReverse.button_pressed):
			joystickSpeedSetpoint -= speedChange
			if (joystickSpeedSetpoint > 0):
				joystickSpeedSetpoint = 0
		else:
			joystickSpeedSetpoint += speedChange
			if (joystickSpeedSetpoint < 0):
				joystickSpeedSetpoint = 0
		
		joystickSpeedSetpoint = clampf(joystickSpeedSetpoint, -maxDrivingSpeed, maxDrivingSpeed)
		$Panel_Controls/HSlider_TargetSpeed.value = joystickSpeedSetpoint

	if (keyboardSpeedChangeTimer >= 0.01):
		if (Input.is_action_pressed("increase_speed")):
			$Panel_Controls/HSlider_TargetSpeed.value += 0.1
			keyboardSpeedChangeTimer = 0
			joystickSpeedSetpoint = $Panel_Controls/HSlider_TargetSpeed.value
		if (Input.is_action_pressed("decrease_speed")):
			$Panel_Controls/HSlider_TargetSpeed.value -= 0.1
			keyboardSpeedChangeTimer = 0
			joystickSpeedSetpoint = $Panel_Controls/HSlider_TargetSpeed.value

	$Tractor.targetSpeed = $Panel_Controls/HSlider_TargetSpeed.value

	steeringWheelPosition = (Input.get_action_strength("joystick_angle_right") - Input.get_action_strength("joystick_angle_left"))

	keyboardAngleChangeTimer += delta

	if ($Panel_JoystickFFBSettings/CheckBox_JoystickAngle.button_pressed):
		$Panel_Controls/HSlider_SteerAngle.value = steeringWheelPosition * 45
	else:
		if (keyboardAngleChangeTimer >= 0.01):
			if (Input.is_action_pressed("increase_steering_angle")):
				$Panel_Controls/HSlider_SteerAngle.value += 0.1
				keyboardAngleChangeTimer = 0
			if (Input.is_action_pressed("decrease_steering_angle")):
				$Panel_Controls/HSlider_SteerAngle.value -= 0.1
				keyboardAngleChangeTimer = 0

	if (Input.is_action_just_pressed("direction_forward")):
		$Panel_JoystickFFBSettings/CheckBox_JoystickReverse.button_pressed = false
		joystickSpeedSetpoint = -joystickSpeedSetpoint

	if (Input.is_action_just_pressed("direction_reverse")):
		$Panel_JoystickFFBSettings/CheckBox_JoystickReverse.button_pressed = true
		joystickSpeedSetpoint = -joystickSpeedSetpoint

	if (!$Panel_Controls/CheckBox_AutomaticSteering.button_pressed):
		$Tractor.steering = -deg_to_rad($Panel_Controls/HSlider_SteerAngle.value)

	timeAfterPAOGIMessage += delta
	timeAfterValidSteerAngleSetpoint += delta
	
	if (timeAfterPAOGIMessage >= 0.0999):
		timeAfterPAOGIMessage = 0
		
		var paogi = "$PAOGI,%s,00000.%d,N,00000.%d,E,4,12,0.01,%1.3f,1.0,%1.3f,%1.2f,%1.2f,,*" % [
			getPAOGITimeString(),
			int($Tractor.latitude * 60e7),
			int($Tractor.longitude * 60e7),
			$Tractor.altitude,
			abs($Tractor.measuredSpeed / 1.852),
			$Tractor.heading,
			$Tractor.roll
			]

		var checksum = getNMEAChecksum(paogi)
		paogi += checksum

		if ($Panel_Controls/CheckBox_DebugMessage_Send. is_pressed()):
			print("Uptime: %1.3f s, IP: %s, port: %d: sent msg: %s" % [float(Time.get_ticks_msec() / 1000.0), udpDestAddress, udpDestPort, paogi])

		paogi += "\r\n"
		clientPeer.put_packet(paogi.to_ascii_buffer())

#		var dummyPacket:String = "$PAOGI,133048.80,6250.0839939,N,02504.6965874,E,4,12,0.70,206.646,0.8,0.019,207.24,-3.98,,*"
#		var checksum = getNMEAChecksum(dummyPacket)
#		dummyPacket += checksum
#		dummyPacket += "\r\n"

#		var dummyPacket:String = "$PAOGI,133048.80,6250.0839939,N,02504.6965874,E,4,12,0.70,206.646,0.8,0.019,207.24,-3.98,,*44\r\n"
#		clientPeer.put_packet(dummyPacket.to_ascii_buffer())
	
#		var testNMEA:String = "$PAOGI,133048.80,6250.0839939,N,02504.6965874,E,4,12,0.70,206.646,0.8,0.019,207.24,-3.98,,*"
#		print(getNMEAChecksum(testNMEA))

	if (timeAfterValidSteerAngleSetpoint > 1):
		$Panel_Controls/CheckBox_AutomaticSteering.button_pressed = false

	updateLocationOrientation(delta)
	handleUDPComms()
	handleForceFeedback($Panel_JoystickFFBSettings/CheckBox_AutoSteeringFFB.button_pressed)

func handleUDPComms():
	# Note: PAOGI is handled separately (in _physics_process) to keep sending of it more precise
	udpServer.poll()

	if (udpServer.is_connection_available()):
		var peer:PacketPeerUDP = udpServer.take_connection()
		peers.append(peer)
	
	for peer in peers:
		var packet:PackedByteArray = peer.get_packet()
		
		var packetDescription:String = "(Not handled)"
		
#		print("Received data: %s" % [packet.hex_encode()])
		
		if (packet.size() > 4 && (packet[0] == 0x80) && (packet[1] == 0x81) && (packet[2] == 0x7F)):
			if (packet[3] == 0xFE):	# 254
				timeAfterValidSteerAngleSetpoint = 0
#				print("Received data: %s" % [packet.hex_encode()])
				var PGN_253:PackedByteArray = [0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC]
				var PGN_253_Size = PGN_253.size() - 1
				
				steerAngleSetpoint = ((int(packet.decode_u8(8) | (packet.decode_s8(9)) << 8))) * 0.01
				var guidanceStatus:int = packet[7];

				packetDescription = "Steer command, setpoint: %1.2f, guidance status: %d" % [steerAngleSetpoint, guidanceStatus]
				
				if (guidanceStatus != 0):
					$Panel_Controls/CheckBox_AutomaticSteering.button_pressed = true
					if ($Panel_JoystickFFBSettings/CheckBox_UseSteeringWheelangleForAutoSteeringAngle.button_pressed):
						$Tractor.steering = deg_to_rad(-steeringWheelPosition * 45)
					else:
						$Tractor.steering = deg_to_rad(-steerAngleSetpoint)
				else:
					$Panel_Controls/CheckBox_AutomaticSteering.button_pressed = false
				
				# Steer angle
				var sa:int = int(-rad_to_deg($Tractor.steering) * 100)
				PGN_253[5] = sa & 0xFF
				PGN_253[6] = (sa >> 8) & 0xFF
				
				# Heading (constant? (code taken from teensy))
				# This is relayed in PAOGI, so not needed here?
				PGN_253[7] = 9999 & 0xFF
				PGN_253[8] = (9999 >> 8) & 0xFF
				
				# Roll (constant? (code taken from teensy))
				# This is relayed in PAOGI, so not needed here?
				PGN_253[9] = 8888 & 0xFF
				PGN_253[10] = (8888 >> 8) & 0xFF
			
				if (steerSwitchPressed):
					PGN_253[11] = 2	# switchByte;
				else:
					PGN_253[11] = 0	# switchByte;
	
				PGN_253[12] = 0	#(uint8_t)pwmDisplay;
				
				# Checksum
				var ck_a:int = 0
				for i in range(2, PGN_253_Size):
					ck_a += PGN_253.decode_u8(i)
				
				PGN_253[PGN_253_Size] = int(ck_a & 0xFF)
			
#				clientPeer.put_packet(PGN_253)
				peer.put_packet(PGN_253)
				
			elif (packet[3] == 200):	# 0xC8
				# Hello from AgIO
				packetDescription = "Hello"
				var helloFromAutoSteer:PackedByteArray = [ 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 ]

				# Steer angle
				var sa:int = int(-rad_to_deg($Tractor.steering) * 100)
				helloFromAutoSteer[5] = sa & 0xFF
				helloFromAutoSteer[6] = (sa >> 8) & 0xFF
				
				var steeringPosition:int = int(-rad_to_deg($Tractor.steering) * 100)
				
				helloFromAutoSteer[7] = steeringPosition & 0xFF	# (uint8_t)helloSteerPosition;
				helloFromAutoSteer[8] = (steeringPosition >> 8) & 0xFF	# helloSteerPosition >> 8;
				helloFromAutoSteer[9] = 0	# switchByte;

				# Checksum (Is this needed? not calculated in teensy code???)
				var ck_a:int = 0
				for i in range(2, helloFromAutoSteer.size()):
					ck_a += helloFromAutoSteer.decode_u8(i)

				helloFromAutoSteer[helloFromAutoSteer.size() - 1] = ck_a

#				clientPeer.put_packet(helloFromAutoSteer)
				peer.put_packet(helloFromAutoSteer)
#				clientPeer_Broadcast.put_packet(helloFromAutoSteer)
				
				# Is this needed?
#				var helloFromIMU:PackedByteArray = [ 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 ]
#				peer.put_packet(helloFromIMU)
				
				
			elif (packet[3] == 202):	# 0xCA
				# Whoami
				packetDescription = "Whoami"
				
				if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202):
					var Eth_myip:PackedByteArray = [192, 168, 5, 126]
					var rem_ip:PackedByteArray = [ 192, 168, 5, 10]
					var scanReply:PackedByteArray = [ 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
						rem_ip[0],rem_ip[1],rem_ip[2], 23 ]

					# Checksum
					var ck_a:int = 0
					for i in range(2, scanReply.size() - 1):
						ck_a += scanReply.decode_u8(i)
					
					scanReply[scanReply.size() - 1] = ck_a;
#					clientPeer_Broadcast.put_packet(scanReply)
					peer.put_packet(scanReply)
			elif (packet[3] == 252):	# 0xFC
				packetDescription = "Steer settings"
				steerSettings_Received.kp = packet.decode_u8(5)
				steerSettings_Received.highPWM = packet.decode_u8(6)
				steerSettings_Received.minPWM = packet.decode_u8(8)
				
				$Panel_SteerSettings/GridContainer_SteerSettings/Label_PGain_Received.text = str(steerSettings_Received.kp)
				$Panel_SteerSettings/GridContainer_SteerSettings/Label_MaximumLimit_Received.text = str(steerSettings_Received.highPWM)
				$Panel_SteerSettings/GridContainer_SteerSettings/Label_MinimumToMove_Received.text = str(steerSettings_Received.minPWM)
				
				steerSettings_ReceivedUpTime_ms = Time.get_ticks_msec()
				
			else:
				packetDescription = "Unhandled packet type %d (0x%X), length: %d" % [packet[3], packet[3], packet.size()]
			
			if ($Panel_Controls/CheckBox_DebugMessage_Receive. is_pressed()):
				var sourceIP:String = peer.get_packet_ip()
				var sourcePort:int = peer.get_packet_port()
				print("Uptime: %1.3f s, IP: %s, port: %d, got msg: %s" % [float(Time.get_ticks_msec() / 1000.0), sourceIP, sourcePort, packetDescription])
		
		
#		peers.append(peer)
		
#		print("Accepted peer: %s:%s" % [peer.get_packet_ip(), peer.get_packet_port()])
#		print("Received data: %s" % [packet.get_string_from_utf8()])

func getNMEAChecksum(sentence:String) -> String:
	var sum:int = 0
	var asciiSentence:PackedByteArray = sentence.to_ascii_buffer()
	var asciiAsterisk:int = "*".to_ascii_buffer()[0]	# Maybe there is a neater way?...
	
	for index in range(1, asciiSentence.size()):
		if (asciiSentence[index] == asciiAsterisk):
			break
		else:
			sum ^= asciiSentence[index]
	
	return "%X%X" % [(sum >> 4), (sum % 16)]

var lastPAOGISecondValue:int = 0
var lastPAOGISubSecondValue:int = 0
	
func getPAOGITimeString() -> String:
	var timeDict = Time.get_time_dict_from_system()
	
	# This sub-second value handling is (just) another dirty hack, sorry...
	# (get_time_dict_from_system doesn't return any more accurate values than seconds)
	lastPAOGISubSecondValue += 10

	if (lastPAOGISubSecondValue >= 100):
		lastPAOGISubSecondValue = 0
	
	if (lastPAOGISecondValue != timeDict.second):
		lastPAOGISecondValue = timeDict.second
		lastPAOGISubSecondValue = 0
	
	var timeString = (getZeroPaddedIntString(timeDict.hour) + 
			getZeroPaddedIntString(timeDict.minute) + 
			getZeroPaddedIntString(timeDict.second) + 
			"." +
			getZeroPaddedIntString(lastPAOGISubSecondValue))
	return timeString

func getZeroPaddedIntString(val:int) -> String:
	# (For NMEA (PAOGI))
	return "%d%d" % [ int(val / 10), int(val % 10) ]

# Handle force feedback outside everything else and only
# create the node when FFB is activated.
# I couldn't get FFB plugin working in Linux (very likely my bad)
# (and likely getting it to work in android would be a pain if even possible).
# This way this should stay usable otherwise.

const ffbDeviceId:int = 0

var ffbNode:FFBPlugin
var ffbInitTried:bool = false
var ffbNodeInitSuccessful:bool = false
var ffbInitSuccessful:bool = false
var ffbEffectId:int

func handleForceFeedback(active:bool):
	var forceFeedbackForce:float = 0
	
	if (active):
		if (!ffbInitTried):
			ffbInitTried = true
			ffbInitSuccessful = initForceFeedback()

		if (ffbInitSuccessful):
			if ($Panel_Controls/CheckBox_AutomaticSteering.button_pressed):
				var steerSetting:SteerSettings = SteerSettings.new()
				
				match ($Panel_SteerSettings/OptionButton_ParametersToUse.selected):
					0:	# Local
						steerSetting.kp = $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_PGain_Local.value
						steerSetting.highPWM = $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MaximumLimit_Local.value
						steerSetting.minPWM = $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MinimumToMove_Local.value
					1:	# Received
						steerSetting = steerSettings_Received
				
				var error = steeringWheelPosition * 45.0 - steerAngleSetpoint
				forceFeedbackForce = calcSteeringPID(error, steerSetting)
				ffbNode.update_constant_force_effect(forceFeedbackForce, 0, ffbEffectId)
#				var targetValue = steerAngleSetpoint / 45.0
#				var error = steeringWheelPosition - targetValue
#				ffbNode.update_constant_force_effect(error * 20.0, 0, ffbEffectId)
			else:
				ffbNode.update_constant_force_effect(0, 0, ffbEffectId)
		
	elif (ffbInitSuccessful):
		destroyForceFeedbackEffect()
		ffbInitSuccessful = false
		ffbInitTried = false
	
	$Panel_JoystickFFBSettings/HSlider_Force.value = forceFeedbackForce
	$Panel_JoystickFFBSettings/Label_Force_Value.text = "%1.2f" % forceFeedbackForce

func initForceFeedback() -> bool:
	if (ffbNode == null):
		ffbNode = FFBPlugin.new()
		self.add_child(ffbNode)
	
	if (!ffbNodeInitSuccessful && ffbNode):
		if ffbNode.init_ffb(ffbDeviceId) < 0: # Initializes the haptic subsystem for given device id
			print("Cant initialize force feedback subsystem, most likely device doesn't support it.")
		else:
			ffbNodeInitSuccessful = true
	
	if (ffbNodeInitSuccessful):
		ffbEffectId = ffbNode.init_constant_force_effect() # Initializes constant force effect and returns its effect_id
		if (ffbEffectId < 0):
			print_debug("Error initialising constant force effect")
		else:
			if (ffbNode.play_constant_force_effect(ffbEffectId, 0) < 0): # Second parameter is how many times the effect is played. 0 == infinite
				print("Starting ffb effect failed")
			else:
				return true

	return false

func destroyForceFeedbackEffect():
	ffbNode.update_constant_force_effect(0, 0, ffbEffectId)
	ffbNode.destroy_ffb_effect(ffbEffectId)

const LOW_HIGH_DEGREES:float = 3.0

# This function is quite straight copy from teensy code
# File: AutosteerPID.c
func calcSteeringPID(steerAngleError:float, steerSettings:SteerSettings) -> float:
	# lowPWM was defined in SteerSettings in teensy-code
	# It looks, however, being updated based on the value of steerSettings.minPWM
	# when receiving new steer setting (PGN 252 / 0xFC).
	# Calculating it here on every round
	var lowPWM:int = clamp(int(steerSettings.minPWM * 1.2), 0, 255)
	
	# Proportional only
	var pValue:float = steerSettings.kp * steerAngleError
	var pwmDrive:int = int(pValue)

	var errorAbs:float = abs(steerAngleError)
	var newMax:int = 0;
	
	var highLowPerDeg = ((float)(steerSettings.highPWM - lowPWM)) / LOW_HIGH_DEGREES

	if (errorAbs < LOW_HIGH_DEGREES):
		newMax = int((errorAbs * highLowPerDeg) + lowPWM);
	else:
		newMax = steerSettings.highPWM;

	# Add min throttle factor so no delay from motor resistance.
	if (pwmDrive < 0):
		pwmDrive -= steerSettings.minPWM;
	elif (pwmDrive > 0):
		pwmDrive += steerSettings.minPWM;

	# Limit the pwm drive
	if (pwmDrive > newMax):
		pwmDrive = newMax;
	if (pwmDrive < -newMax):
		pwmDrive = -newMax;

	return clamp(pwmDrive / 255.0, -1, 1)
	
const configFileName:String = "res://AgOpenGPSPoC.cfg"
	
func loadConfig():
	var config:ConfigFile = ConfigFile.new()

	if (config.load(configFileName) != OK):
		print("Config file not found, using defaults.")
		return

	$Panel_Controls/CheckBox_DebugMessage_Receive.button_pressed = config.get_value("PrintDebugMessages", "Receive")
	$Panel_Controls/CheckBox_DebugMessage_Send.button_pressed = config.get_value("PrintDebugMessages", "Send")

	$Panel_Controls/CheckBox_ShowJoystickSettings.button_pressed = config.get_value("Visibility", "JoystickSettings")

	$Panel_JoystickFFBSettings/CheckBox_JoystickAngle.button_pressed = config.get_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_Manual")
	$Panel_JoystickFFBSettings/CheckBox_JoystickSpeed.button_pressed = config.get_value("Joystick", "UseJoystickOrPedalsForSpeed")
	$Panel_JoystickFFBSettings/CheckBox_AutoSteeringFFB.button_pressed = config.get_value("ForceFeedback", "AutoSteeringFFB")
	$Panel_JoystickFFBSettings/CheckBox_UseSteeringWheelangleForAutoSteeringAngle.button_pressed = config.get_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_AutomaticSteering")
	$Panel_JoystickFFBSettings/CheckBox_ShowSteerSettings.button_pressed = config.get_value("Visibility", "ShowSteerSettings")
	
	$Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_PGain_Local.value = config.get_value("SteerSettings_Local", "PGain")
	$Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MaximumLimit_Local.value = config.get_value("SteerSettings_Local", "MaximumLimit")
	$Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MinimumToMove_Local.value = config.get_value("SteerSettings_Local", "MinimumToMove")
	
	$Panel_SteerSettings/GridContainer_SteerSettings/Label_PGain_Received.text = config.get_value("SteerSettings_Received", "PGain")
	$Panel_SteerSettings/GridContainer_SteerSettings/Label_MaximumLimit_Received.text = config.get_value("SteerSettings_Received", "MaximumLimit")
	$Panel_SteerSettings/GridContainer_SteerSettings/Label_MinimumToMove_Received.text = config.get_value("SteerSettings_Received", "MinimumToMove")
	
func saveConfig():
	var config:ConfigFile = ConfigFile.new()

	config.set_value("PrintDebugMessages", "Receive", $Panel_Controls/CheckBox_DebugMessage_Receive.button_pressed)
	config.set_value("PrintDebugMessages", "Send", $Panel_Controls/CheckBox_DebugMessage_Send.button_pressed)

	config.set_value("Visibility", "JoystickSettings", $Panel_Controls/CheckBox_ShowJoystickSettings.button_pressed)

	config.set_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_Manual", $Panel_JoystickFFBSettings/CheckBox_JoystickAngle.button_pressed)
	config.set_value("Joystick", "UseJoystickOrPedalsForSpeed", $Panel_JoystickFFBSettings/CheckBox_JoystickSpeed.button_pressed)
	config.set_value("ForceFeedback", "AutoSteeringFFB", $Panel_JoystickFFBSettings/CheckBox_AutoSteeringFFB.button_pressed)
	config.set_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_AutomaticSteering", $Panel_JoystickFFBSettings/CheckBox_UseSteeringWheelangleForAutoSteeringAngle.button_pressed)
	config.set_value("Visibility", "ShowSteerSettings", $Panel_JoystickFFBSettings/CheckBox_ShowSteerSettings.button_pressed)

	config.set_value("SteerSettings_Local", "PGain", $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_PGain_Local.value)
	config.set_value("SteerSettings_Local", "MaximumLimit", $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MaximumLimit_Local.value)
	config.set_value("SteerSettings_Local", "MinimumToMove", $Panel_SteerSettings/GridContainer_SteerSettings/SpinBox_MinimumToMove_Local.value)

	config.set_value("SteerSettings_Received", "PGain", $Panel_SteerSettings/GridContainer_SteerSettings/Label_PGain_Received.text)
	config.set_value("SteerSettings_Received", "MaximumLimit", $Panel_SteerSettings/GridContainer_SteerSettings/Label_MaximumLimit_Received.text)
	config.set_value("SteerSettings_Received", "MinimumToMove", $Panel_SteerSettings/GridContainer_SteerSettings/Label_MinimumToMove_Received.text)
	
	config.save(configFileName)


func _on_spin_box_max_speed_value_changed(value):
	maxDrivingSpeed = value
	$Panel_Controls/HSlider_TargetSpeed.min_value = -value
	$Panel_Controls/HSlider_TargetSpeed.max_value = value
