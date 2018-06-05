#version 400 core
in vec2 TexCoords;

const float gaussWeightsSigma3[7] =float[7]
(
	0.106595f,
	0.140367f,
	0.165569f,
	0.174938f,
	0.165569f,
	0.140367f,
	0.106595f
);

out float fragColor;

uniform sampler2D ssaoInput;
uniform sampler2D depthInput;

uniform vec2 texelSize;

void main() 
{
	float sum = 0.0f;
	float weightsSum = 0.0f;

	float depth = texture(depthInput, TexCoords).r;

	for (int i = -3; i <= 3; i++) {
		vec2 sampleTexCoord = TexCoords + i * texelSize;
		float sampleDepth = texture(depthInput, sampleTexCoord).r;

		float depthsDiff = 0.1f * abs(depth - sampleDepth);
		depthsDiff *= depthsDiff;
		float weight = 1.0f / (depthsDiff + 0.001f);
		weight *= gaussWeightsSigma3[3 + i];

		sum += weight * texture(ssaoInput, sampleTexCoord).r;
		weightsSum += weight;
	}
   
	fragColor = sum / weightsSum;
}