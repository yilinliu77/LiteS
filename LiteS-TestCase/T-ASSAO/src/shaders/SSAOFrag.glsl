#version 400 core
out float FragColor;

in vec2 TexCoords;

const float GOLDEN_ANGLE = 2.4f;
const int SAMPLESCOUNT = 16;
const float TWOPI = 6.28f;

uniform sampler2D gDepth;
uniform sampler2D gNormal;
uniform sampler2D texNoise;

uniform float gNearPlaneHeightNormalized; // at distance 1 from the eye
uniform float gAspect;
uniform float gRadiusWorld;
uniform float gMaxRadiusScreen;
uniform float gContrast;

float InterleavedGradientNoise(vec2 vPositionScreen) {
	vec3 Magic = vec3(0.06711056f, 4.0f*0.00583715f, 52.9829189f);
	return fract(Magic.z * fract(dot(vPositionScreen, Magic.xy)));
}

float AlchemyNoise(vec2 vPositionScreen) {
	return 30.0f*pow(vPositionScreen.x,vPositionScreen.y) + 10.0f*(vPositionScreen.x*vPositionScreen.y);
}

vec2 VogelDiskOffset(int sampleIndex, float phi) {
	float r = sqrt(sampleIndex + 0.5f) / sqrt(SAMPLESCOUNT);
	float theta = sampleIndex * GOLDEN_ANGLE + phi;

	return vec2(r * cos(theta), r * sin(theta));
}

vec2 AlchemySpiralOffset(int sampleIndex, float phi) {
	float alpha = float(sampleIndex + 0.5f) / SAMPLESCOUNT;
	float theta = 7.0f*TWOPI*alpha + phi;

	return vec2(cos(theta), sin(theta));
}

vec3 Position_View(vec2 texCoord) {
	vec3 position = vec3(texCoord*2,-1.0f);
	position.xy = position.xy - 1.0f;
	position.y *= gNearPlaneHeightNormalized;
	position.x *= gNearPlaneHeightNormalized * gAspect;
	position *= -texture(gDepth, texCoord).r;
	return position;
}

void main()
{
	vec3 Position = Position_View(TexCoords);
	vec3 Normal = texture(gNormal, TexCoords).rgb;

	float Noise = InterleavedGradientNoise(gl_FragCoord.xy);
	float ANoise = AlchemyNoise(gl_FragCoord.xy);

	vec2 RadiusScreen = vec2(gRadiusWorld, gRadiusWorld) / Position.z;
	RadiusScreen = min(RadiusScreen, gMaxRadiusScreen);
	RadiusScreen.x *= gAspect;

	float ao = 0.0f;
	vec3 dummy;

	for (int i = 0; i < SAMPLESCOUNT; ++i) {
		vec2 SampleOffset = vec2(0.0f,0.0f);
		SampleOffset = VogelDiskOffset(i, TWOPI*Noise);
		//SampleOffset = AlchemySpiralOffset(i, ANoise);
		
		vec2 SampleTexCoord = TexCoords + RadiusScreen * SampleOffset;

		vec3 SamplePosition = Position_View(SampleTexCoord);

		vec3 v = SamplePosition - Position;

		ao += max(0.0f, dot(v, Normal) + 0.002f*Position.z) / (dot(v, v) + 0.001f);
		dummy = v;
	}

	ao = clamp(ao / SAMPLESCOUNT,0.0,1.0);
	ao = 1.0f - ao;
	ao = pow(ao, gContrast);

	FragColor = ao;
	//FragColor = vec4(ao,ao,ao,ao);
	//FragColor = vec4(dummy, 1.0f);
}