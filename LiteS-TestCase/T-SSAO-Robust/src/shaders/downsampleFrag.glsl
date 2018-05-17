#version 400 core
in vec2 TexCoords;

out float fragColor;

uniform sampler2D tex1;

uniform vec2 gProjParams;

void main() {
	vec2 texCoord = TexCoords + vec2(-0.25f, -0.25f)*vec2(2 / 1280, 2 / 720);

	fragColor = texture(tex1, TexCoords).r;
}