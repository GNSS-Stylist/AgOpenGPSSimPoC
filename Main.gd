extends Node3D

@onready var terrain = $Terrain
@onready var collisionShape = $Terrain/StaticBody3D/CollisionShape3D

var udpServer:UDPServer
var clientPeer:PacketPeerUDP
var clientPeer_Broadcast:PacketPeerUDP
var peers = []

const udpServerPort:int = 8888
const udpDestPort:int = 9999

# Called when the node enters the scene tree for the first time.
func _ready():
# This does not work here (handled in _process instead):
#	updateTerrainCollisionShape()

	clientPeer = PacketPeerUDP.new()
	clientPeer.set_broadcast_enabled(true)
	clientPeer.set_dest_address("192.168.5.10", udpDestPort)

	clientPeer_Broadcast = PacketPeerUDP.new()
	clientPeer_Broadcast.set_broadcast_enabled(true)
	clientPeer_Broadcast.set_dest_address("255.255.255.255", 9999)

	udpServer = UDPServer.new()
	
	udpServer.listen(udpServerPort)

func updateTerrainCollisionShape():
	var terrainMaterial:ShaderMaterial = terrain.material_override
	var heightmapTexture:Texture2D = terrainMaterial.get_shader_parameter("heightmap")
	var heightMultiplier:float = terrainMaterial.get_shader_parameter("heightMultiplier")
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
	shape.map_data = data
	collisionShape.shape = shape
	var scale = 1000/float(heightMapImage.get_width())
	collisionShape.scale = Vector3(scale, 1, scale)

var accumDelta:float = 0
var terrainCollisionShapeUpdated:bool = false

var keyboardSpeedChangeTimer:float = 0
var keyboardAngleChangeTimer:float = 0
var steerSwitchPressed:bool = false

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	accumDelta += delta
	if (!terrainCollisionShapeUpdated):
		# This is very dirty way to make sure the shadermatererial's
		# get_shader_parameter-function gives sensible values before
		# using them in updateTerrainCollisionShape
		# (updateTerrainCollisionShape does not work when called from _ready)
		updateTerrainCollisionShape()
		terrainCollisionShapeUpdated = true
	
	if (Input.is_action_just_pressed("reset_angle_and_speed")):
		_on_button_reset_steer_angle_and_speed_pressed()

	if (Input.is_action_just_pressed("steer_switch")):
		steerSwitchPressed = !steerSwitchPressed
		
	if (Input.is_action_just_pressed("full_screen_toggle")):
		if (DisplayServer.window_get_mode() == DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN):
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
		else:
			DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)

	keyboardSpeedChangeTimer += delta
	keyboardAngleChangeTimer += delta

	if (keyboardSpeedChangeTimer >= 0.1):
		if (Input.is_action_pressed("increase_speed")):
			$Panel_Controls/HSlider_TargetSpeed.value += 1
			keyboardSpeedChangeTimer = 0
		if (Input.is_action_pressed("decrease_speed")):
			$Panel_Controls/HSlider_TargetSpeed.value -= 1
			keyboardSpeedChangeTimer = 0

	if (keyboardAngleChangeTimer >= 0.1):
		if (Input.is_action_pressed("increase_steering_angle")):
			$Panel_Controls/HSlider_SteerAngle.value += 1
			keyboardAngleChangeTimer = 0
		if (Input.is_action_pressed("decrease_steering_angle")):
			$Panel_Controls/HSlider_SteerAngle.value -= 1
			keyboardAngleChangeTimer = 0

	$Tractor.targetSpeed = $Panel_Controls/HSlider_TargetSpeed.value

	if (!$Panel_Controls/CheckBox_AutomaticSteering.button_pressed):
		$Tractor.steering = -deg_to_rad($Panel_Controls/HSlider_SteerAngle.value)

	cameraSwitch()
	updateLocationOrientation()
	handleUDPComms()
	
func cameraSwitch():
	if (Input.is_action_just_pressed("camera_flyer")):
		$FirstPersonFlyer.set_LocationOrientation(self.transform.affine_inverse() * get_viewport().get_camera_3d().get_global_transform())
		$FirstPersonFlyer/Head/FirstPersonCamera.current = true
	elif (Input.is_action_just_pressed("camera_tractor_driver")):
		$Tractor/Camera_Driver.current = true
	elif (Input.is_action_just_pressed("camera_tractor_follow")):
		$Tractor/Camera_Follow.current = true
	elif (Input.is_action_just_pressed("camera_tractor_angled")):
		$Tractor/Camera_Angled.current = true
		
func updateLocationOrientation():
	$Panel_LocationOrientation/Label_Latitude_Value.text = "%1.7f N" % $Tractor.latitude
	$Panel_LocationOrientation/Label_Longitude_Value.text = "%1.7f E" % $Tractor.longitude
	$Panel_LocationOrientation/Label_Altitude_Value.text = "%1.2f m" % $Tractor.altitude
	$Panel_LocationOrientation/Label_Heading_Value.text = "%1.1f" % $Tractor.heading
	$Panel_LocationOrientation/Label_Roll_Value.text = "%1.1f" % $Tractor.roll
	$Panel_LocationOrientation/Label_Speed_Value.text = "%1.2f" % $Tractor.measuredSpeed

func _on_h_slider_steer_angle_value_changed(value):
	$Panel_Controls/Label_SteerAngle_Value.text = "%d" % value

func _on_h_slider_target_speed_value_changed(value):
	$Panel_Controls/Label_TargetSpeed_Value.text = "%d" % value

func _on_button_reset_steer_angle_and_speed_pressed():
	$Panel_Controls/HSlider_SteerAngle.value = 0
	$Panel_Controls/HSlider_TargetSpeed.value = 0

var timeAfterPAOGIMessage:float = 0
var timeAfterValidSteerAngleSetpoint:float = 0

func _physics_process(delta):
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
		paogi += "\r\n"
		clientPeer.put_packet(paogi.to_ascii_buffer())

#		print(paogi)

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

func handleUDPComms():
	# Note: PAOGI is handled separately (in _physics_process) to keep sending of it more precise
	udpServer.poll()

	if (udpServer.is_connection_available()):
		var peer:PacketPeerUDP = udpServer.take_connection()
		peers.append(peer)
	
	for peer in peers:
		var packet:PackedByteArray = peer.get_packet()
		
#		print("Received data: %s" % [packet.hex_encode()])
		
		if (packet.size() > 4 && (packet[0] == 0x80) && (packet[1] == 0x81) && (packet[2] == 0x7F)):
			if (packet[3] == 0xFE):
				timeAfterValidSteerAngleSetpoint = 0
#				print("Received data: %s" % [packet.hex_encode()])
				var PGN_253:PackedByteArray = [0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC]
				var PGN_253_Size = PGN_253.size() - 1
				
				var steerAngleSetpoint:float = ((int(packet.decode_u8(8) | (packet.decode_s8(9)) << 8))) * 0.01
				var guidanceStatus:int = packet[7];

#				print("Received steer command, setpoint: %1.2f, guidance status: %d" % [steerAngleSetpoint, guidanceStatus])
				
				if (guidanceStatus != 0):
					$Panel_Controls/CheckBox_AutomaticSteering.button_pressed = true
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
				
			elif (packet[3] == 200):
				# Hello from AgIO
#				print("Received hello")
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
				
				
			elif (packet[3] == 202):
				# Whoami
				print("Received whoami")
				
				if (packet[4] == 3 && packet[5] == 202 && packet[6] == 202):
					var Eth_myip:PackedByteArray = [192, 168, 5, 126]
					var rem_ip:PackedByteArray = [ 192, 168, 5, 10]
					var scanReply = [ 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
						rem_ip[0],rem_ip[1],rem_ip[2], 23 ]

					# Checksum
					var ck_a:int = 0
					for i in range(2, scanReply.size() - 1):
						ck_a += scanReply.decode_u8(i)
					
					scanReply[scanReply.size() - 1] = ck_a;
#					clientPeer_Broadcast.put_packet(scanReply)
					peer.put_packet(scanReply)
#			else:
#				print("Unhandled packet type %d (0x%X)" % [packet[3], packet[3]])
		
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
