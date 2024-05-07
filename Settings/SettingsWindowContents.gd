extends TabContainer

# Replaced loading/saving here with main window calls
# (not sure about creation/exit_tree-order)
#func _ready():
#	loadConfig()

#func _exit_tree():
#	saveConfig()

const configFileName:String = "res://AgOpenGPSPoC.cfg"

var show3rdPartyCreditsOnStart:bool

func loadConfig():
	var config:ConfigFile = ConfigFile.new()

	if (config.load(configFileName) != OK):
		print("Config file not found, using defaults.")

	$General/VBoxContainer_General/CheckBox_DebugMessage_Receive.button_pressed = config.get_value("PrintDebugMessages", "Receive", false)
	$General/VBoxContainer_General/CheckBox_DebugMessage_Send.button_pressed = config.get_value("PrintDebugMessages", "Send", false)
	$General/VBoxContainer_General/CheckBox_ViewLocationOrientation.button_pressed = config.get_value("Visibility", "ViewLocationOrientation", false)
	$General/VBoxContainer_General/CheckBox_ViewFFBJoystickInfo.button_pressed = config.get_value("Visibility", "ViewFFBAndJoystickInfo", false)
	$General/VBoxContainer_General/CheckBox_Shadows.button_pressed = config.get_value("Visibility", "RenderShadows", true)

	$Controls/VBoxContainer_Controls/CheckBox_JoystickAngle.button_pressed = config.get_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_Manual", false)
	$Controls/VBoxContainer_Controls/CheckBox_JoystickSpeed.button_pressed = config.get_value("Joystick", "UseJoystickOrPedalsForSpeed", false)
	$Controls/VBoxContainer_Controls/CheckBox_AutoSteeringFFB.button_pressed = config.get_value("ForceFeedback", "AutoSteeringFFB", false)
	$Controls/VBoxContainer_Controls/CheckBox_UseSteeringWheelangleForAutoSteeringAngle.button_pressed = config.get_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_AutomaticSteering", false)
	
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_PGain_Local.value = config.get_value("SteerSettings_Local", "PGain", 50)
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_MaximumLimit_Local.value = config.get_value("SteerSettings_Local", "MaximumLimit", 255)
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_MinimumToMove_Local.value = config.get_value("SteerSettings_Local", "MinimumToMove", 0)
	
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_PGain_Received.text = config.get_value("SteerSettings_Received", "PGain", "0")
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_MaximumLimit_Received.text = config.get_value("SteerSettings_Received", "MaximumLimit", "0")
	$SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_MinimumToMove_Received.text = config.get_value("SteerSettings_Received", "MinimumToMove", "0")
	$SteerSettings/VBoxContainer/HBoxContainer/OptionButton_ParametersToUse.selected = config.get_value("SteerSettings", "ParametersToUse", 0)

	$Machine/VBoxContainer_Visibilities/CheckBox_HideObstacles.button_pressed = config.get_value("Visibility", "HideObstacles", false)
	$Machine/VBoxContainer_Visibilities/CheckBox_OnlyWheels.button_pressed = config.get_value("Visibility", "OnlyWheels", false)
	$Machine/VBoxContainer_Visibilities/CheckBox_ImplementVisible.button_pressed = config.get_value("Visibility", "Implement", true)
	$Machine/VBoxContainer_Visibilities/CheckBox_ImplementColliding.button_pressed = config.get_value("Implement", "Colliding", true)
	$Machine/VBoxContainer_Visibilities/HBoxContainer_ImplementWidth/SpinBox_ImplementWidth.value = config.get_value("Implement", "Width", 10)
	$Machine/VBoxContainer_Visibilities/CheckBox_CrosshairVisible.button_pressed = config.get_value("Visibility", "TractorCrosshair", false)

	$Terrain/GridContainer/SpinBox_RandomSeed.value = config.get_value("Terrain", "RandomSeed", 3)
	$Terrain/GridContainer/SpinBox_HeightMultiplier.value = config.get_value("Terrain", "HeightMultiplier", 50)

	$UDP/GridContainer/SpinBox_IP_1.value = config.get_value("UDP", "Subnet_1", 192)
	$UDP/GridContainer/SpinBox_IP_2.value = config.get_value("UDP", "Subnet_2", 168)
	$UDP/GridContainer/SpinBox_IP_3.value = config.get_value("UDP", "Subnet_3", 5)

	show3rdPartyCreditsOnStart = config.get_value("Visibility", "Show3rdPartyCreditsOnStart", true)
	
func saveConfig():
	var config:ConfigFile = ConfigFile.new()

	config.set_value("PrintDebugMessages", "Receive", $General/VBoxContainer_General/CheckBox_DebugMessage_Receive.button_pressed)
	config.set_value("PrintDebugMessages", "Send", $General/VBoxContainer_General/CheckBox_DebugMessage_Send.button_pressed)
	config.set_value("Visibility", "ViewLocationOrientation", $General/VBoxContainer_General/CheckBox_ViewLocationOrientation.button_pressed)
	config.set_value("Visibility", "ViewFFBAndJoystickInfo", $General/VBoxContainer_General/CheckBox_ViewFFBJoystickInfo.button_pressed)
	config.set_value("Visibility", "RenderShadows", $General/VBoxContainer_General/CheckBox_Shadows.button_pressed)

	config.set_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_Manual", $Controls/VBoxContainer_Controls/CheckBox_JoystickAngle.button_pressed)
	config.set_value("Joystick", "UseJoystickOrPedalsForSpeed", $Controls/VBoxContainer_Controls/CheckBox_JoystickSpeed.button_pressed)
	config.set_value("ForceFeedback", "AutoSteeringFFB", $Controls/VBoxContainer_Controls/CheckBox_AutoSteeringFFB.button_pressed)
	config.set_value("Joystick", "UseJoystickOrSteeringWheelForWheelangle_AutomaticSteering", $Controls/VBoxContainer_Controls/CheckBox_UseSteeringWheelangleForAutoSteeringAngle.button_pressed)

	config.set_value("SteerSettings_Local", "PGain", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_PGain_Local.value)
	config.set_value("SteerSettings_Local", "MaximumLimit", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_MaximumLimit_Local.value)
	config.set_value("SteerSettings_Local", "MinimumToMove", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/SpinBox_MinimumToMove_Local.value)

	config.set_value("SteerSettings_Received", "PGain", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_PGain_Received.text)
	config.set_value("SteerSettings_Received", "MaximumLimit", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_MaximumLimit_Received.text)
	config.set_value("SteerSettings_Received", "MinimumToMove", $SteerSettings/VBoxContainer/GridContainer_SteerSettings/Label_MinimumToMove_Received.text)
	config.set_value("SteerSettings", "ParametersToUse", $SteerSettings/VBoxContainer/HBoxContainer/OptionButton_ParametersToUse.selected)
	
	config.set_value("Visibility", "HideObstacles", $Machine/VBoxContainer_Visibilities/CheckBox_HideObstacles.button_pressed)
	config.set_value("Visibility", "OnlyWheels", $Machine/VBoxContainer_Visibilities/CheckBox_OnlyWheels.button_pressed)
	config.set_value("Visibility", "Implement", $Machine/VBoxContainer_Visibilities/CheckBox_ImplementVisible.button_pressed)
	config.set_value("Implement", "Colliding", $Machine/VBoxContainer_Visibilities/CheckBox_ImplementColliding.button_pressed)
	config.set_value("Implement", "Width", $Machine/VBoxContainer_Visibilities/HBoxContainer_ImplementWidth/SpinBox_ImplementWidth.value)
	config.set_value("Visibility", "TractorCrosshair", $Machine/VBoxContainer_Visibilities/CheckBox_CrosshairVisible.button_pressed)

	config.set_value("Terrain", "RandomSeed", $Terrain/GridContainer/SpinBox_RandomSeed.value)
	config.set_value("Terrain", "HeightMultiplier", $Terrain/GridContainer/SpinBox_HeightMultiplier.value)
	
	config.set_value("Visibility", "Show3rdPartyCreditsOnStart", show3rdPartyCreditsOnStart)

	config.set_value("UDP", "Subnet_1", $UDP/GridContainer/SpinBox_IP_1.value)
	config.set_value("UDP", "Subnet_2", $UDP/GridContainer/SpinBox_IP_2.value)
	config.set_value("UDP", "Subnet_3", $UDP/GridContainer/SpinBox_IP_3.value)

	config.save(configFileName)


func _on_button_update_udp_pressed():
	var subnet:Array[int] = [
		int($UDP/GridContainer/SpinBox_IP_1.value),
		int($UDP/GridContainer/SpinBox_IP_2.value),
		int($UDP/GridContainer/SpinBox_IP_3.value)
		] 
	
	get_parent().get_parent().updateUDPSubnet(subnet)
