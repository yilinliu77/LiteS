#version 330 core
layout (location = 0) out vec4 gPositionDepth;
layout (location = 1) out vec3 gNormal;
layout (location = 2) out vec4 gAlbedoSpec;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;

uniform vec2 gProjParams; 
uniform sampler2D texture_diffuse1;

float LinearizeDepth(float vDepth) {
	float Z = vDepth * 2.0 - 1.0;
	return (2.0 * gProjParams[0] * gProjParams[1]) / (gProjParams[1] + gProjParams[0] - Z * (gProjParams[1] - gProjParams[0]));
}

void main()
{    
	// Store the fragment position vector in the first gbuffer texture
	gPositionDepth.xyz = FragPos;
	// And store linear depth into gPositionDepth's alpha component
	gPositionDepth.a = LinearizeDepth(gl_FragCoord.z); // Divide by FAR if you need to store depth in range 0.0 - 1.0 (if not using floating point colorbuffer)
	// Also store the per-fragment normals into the gbuffer
	gNormal = normalize(Normal);
	// And the diffuse per-fragment color
	gAlbedoSpec = texture(texture_diffuse1, TexCoords);
}