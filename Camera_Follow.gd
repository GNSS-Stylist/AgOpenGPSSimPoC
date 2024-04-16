extends Camera3D

# This camera tracker is cloned from Godot's example project "Truck Town"

@export var min_distance = 2.0
@export var max_distance = 4.0
@export var angle_v_adjust = 0.0
@export var height = 1.5
@export var node_to_follow:NodePath

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	var target = get_node(node_to_follow).get_global_transform().origin
	var pos = get_global_transform().origin

	var from_target = pos - target

	# Check ranges.
	if from_target.length() < min_distance:
		from_target = from_target.normalized() * min_distance
	elif from_target.length() > max_distance:
		from_target = from_target.normalized() * max_distance

	from_target.y = height

	pos = target + from_target

	look_at_from_position(pos, target, Vector3.UP)

	# Turn a little up or down
	var t = get_transform()
	t.basis = Basis(t.basis[0].normalized(), deg_to_rad(angle_v_adjust)) * t.basis
	set_transform(t)









