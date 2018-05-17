#version 400 core
in vec2 TexCoords;

out vec4 fragColor;

uniform sampler2D depthTexture;
uniform sampler2D quaterDepthTexture;
uniform sampler2D ssaoTexture;

uniform vec2 gProjParams;
uniform vec2 gPixelSize;

float DepthNDCToView(float vDepthNdc) {
	return -gProjParams.y / (vDepthNdc + gProjParams.x);
}

float LinearizeDepth(float vDepth) {
	float Z = vDepth * 2.0 - 1.0; // Back to NDC 
	return (2.0 * gProjParams[0] * gProjParams[1]) / (gProjParams[1] + gProjParams[0] - Z * (gProjParams[1] - gProjParams[0]));
}

void main() {
	float Depth = texture(depthTexture, TexCoords).r;
	Depth = DepthNDCToView(Depth);
	vec4 QuaterDepth = textureGather(quaterDepthTexture, TexCoords, 0);
	vec4 DepthsDiffs = abs(vec4(Depth) - QuaterDepth);

	vec4 QuaterSSAO = textureGather(ssaoTexture, TexCoords, 0);

	vec2 imageCoord = TexCoords / gPixelSize;
	vec2 fractional = fract(imageCoord);
	float a = (1.0f - fractional.x) * (1.0f - fractional.y);
	float b = fractional.x * (1.0f - fractional.y);
	float c = (1.0f - fractional.x) * fractional.y;
	float d = fractional.x * fractional.y;

	vec4 ssao = vec4(0.0f);
	float weightsSum = 0.0f;

	float weight00 = a / (DepthsDiffs.x + 0.001f);
	ssao += weight00 * QuaterSSAO.x;
	weightsSum += weight00;

	float weight10 = b / (DepthsDiffs.y + 0.001f);
	ssao += weight10 * QuaterSSAO.y;
	weightsSum += weight10;

	float weight01 = c / (DepthsDiffs.z + 0.001f);
	ssao += weight01 * QuaterSSAO.z;
	weightsSum += weight01;

	float weight11 = d / (DepthsDiffs.w + 0.001f);
	ssao += weight11 * QuaterSSAO.w;
	weightsSum += weight11;

	ssao /= weightsSum;

	fragColor = ssao;
}