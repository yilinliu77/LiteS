#version 400 core
in vec2 TexCoords;

layout(location = 0) out float gQuadDepth1;
layout(location = 1) out float gQuadDepth2;
layout(location = 2) out float gQuadDepth3;
layout(location = 3) out float gQuadDepth4;

uniform sampler2D tex1;

uniform vec2 gProjParams;

void main() {
	vec2 pixelOffset = vec2(1.0f / 1280.0f, 1.0f / 720.0f);

	vec2 texCoord1 = TexCoords + vec2(-0.25f, -0.25f)*vec2(2.0f / 1280.0f, 2.0f / 720.0f);
	vec2 texCoord2 = TexCoords + vec2(0.25f, -0.25f)*vec2(2.0f / 1280.0f, 2.0f / 720.0f);
	vec2 texCoord3 = TexCoords + vec2(-0.25f, 0.25f)*vec2(2.0f / 1280.0f, 2.0f / 720.0f);
	vec2 texCoord4 = TexCoords + vec2(0.25f, 0.25f)*vec2(2.0f / 1280.0f, 2.0f / 720.0f);

	gQuadDepth1 = texture(tex1, texCoord1).r;
	gQuadDepth2 = texture(tex1, texCoord2).r;
	gQuadDepth3 = texture(tex1, texCoord3).r;
	gQuadDepth4 = texture(tex1, texCoord4).r;
}