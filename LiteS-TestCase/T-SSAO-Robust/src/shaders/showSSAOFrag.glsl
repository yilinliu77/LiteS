#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D gSSAO;

void main() {
	FragColor = vec4(vec3(texture(gSSAO, TexCoords).r), 1.0);
}