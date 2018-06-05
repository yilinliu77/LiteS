#version 400 core
layout (location = 0) out float gDepth;
layout (location = 1) out vec3 gNormal;
layout (location = 2) out vec4 gAlbedoSpec;
layout(location = 3) out vec4 gPosition;


in vec2 TexCoords;
in vec4 FragPos;
in vec3 Normal;

uniform sampler2D texture_diffuse1;
uniform vec2 gProjParams;


//Both ok
float LinearizeDepth(float vDepth)
{
    float Z = vDepth * 2.0 - 1.0; 
    return (2.0 * gProjParams[0] * gProjParams[1]) / (gProjParams[1] + gProjParams[0] - Z * (gProjParams[1] - gProjParams[0]));
}

float DepthNDCToView(float vDepthNdc) {
	vDepthNdc = 2 * vDepthNdc - 1;
	return -gProjParams.y / (vDepthNdc + gProjParams.x);
}

void main()
{    
	//gDepth.xyz = FragPos;
	gDepth = DepthNDCToView(gl_FragCoord.z);
	gPosition=FragPos;
	//gDepth = gl_FragCoord.z;
    gNormal = normalize(Normal);
	gAlbedoSpec= texture(texture_diffuse1, TexCoords); // Currently all objects have constant albedo color
}