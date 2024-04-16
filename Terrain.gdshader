shader_type spatial;

render_mode cull_disabled;

uniform sampler2D heightmap : repeat_disable;
uniform float heightMultiplier = 1.0;
//uniform vec4 insideFieldAlbedo : source_color = vec4(1.0);
//uniform vec4 outsideFieldAlbedo : source_color = vec4(1.0);
uniform sampler2D terrainTexture;
uniform sampler2D fieldsTexture;
uniform vec2 terrainTextureScale = vec2(10.0);
//uniform float fieldRadius = 0.1;
uniform float gridDist = 0.01;
uniform float gridWidth = 0.02;
uniform float edgeRise = 100.0;

void vertex() {
	VERTEX.y += texture(heightmap, UV).r * heightMultiplier;
	if ((UV.x < 0.001) || (UV.x > 0.999) || (UV.y < 0.001) || (UV.y > 0.999)) {
		VERTEX.y += edgeRise;
//		UV = vec2(0.0, 0.0);
	}
}

void fragment() {
/*	float distFromCenter = distance(UV,vec2(0.5,0.5));
	if (distFromCenter < fieldRadius) {
		ALBEDO = insideFieldAlbedo.rgb * texture(terrainTexture, UV * terrainTextureScale).rgb;
	} else {
		ALBEDO = outsideFieldAlbedo.rgb * texture(terrainTexture, UV * terrainTextureScale).rgb;
	}
*/
	
	float fieldFraction = texture(fieldsTexture, UV).r;
	
//	ALBEDO = mix(outsideFieldAlbedo.rgb, insideFieldAlbedo.rgb, texture(fieldsTexture, UV).r) * texture(terrainTexture, UV * terrainTextureScale).rgb;

	vec3 fieldAlbedo = texture(fieldsTexture, UV).rgb * texture(terrainTexture, UV * terrainTextureScale).rgb;

//	ALBEDO = texture(fieldsTexture, UV).rgb * texture(terrainTexture, UV * terrainTextureScale).rgb;
	ALBEDO = mix(texture(terrainTexture, UV * terrainTextureScale).rgb, fieldAlbedo, texture(fieldsTexture, UV).a);
	
	if (fract(UV.x / gridDist) < gridWidth) {
		ALBEDO.r = 1.0;
	}
	
	if (fract(UV.y / gridDist) < gridWidth) {
		ALBEDO.b = 1.0;
	}
}
