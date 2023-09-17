shader_type spatial;

uniform sampler2D heightmap : repeat_disable;
uniform float heightMultiplier = 1.0;
uniform vec4 insideFieldAlbedo : source_color = vec4(1.0);
uniform vec4 outsideFieldAlbedo : source_color = vec4(1.0);
uniform sampler2D terrainTexture;
uniform vec2 textureScale = vec2(1.0);
uniform float fieldRadius = 0.1;
uniform float gridDist = 0.01;
uniform float gridWidth = 0.02;

void vertex() {
	VERTEX.y += texture(heightmap, UV).r * heightMultiplier;
}

void fragment() {
	float distFromCenter = distance(UV,vec2(0.5,0.5));
	if (distFromCenter < fieldRadius) {
		ALBEDO = insideFieldAlbedo.rgb * texture(terrainTexture, UV * textureScale).rgb;
	} else {
		ALBEDO = outsideFieldAlbedo.rgb * texture(terrainTexture, UV * textureScale).rgb;
	}
	
	if (fract(UV.x / gridDist) < gridWidth) {
		ALBEDO.r = 1.0;
	}
	
	if (fract(UV.y / gridDist) < gridWidth) {
		ALBEDO.b = 1.0;
	}
}
