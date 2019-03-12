#version 330 core

out vec4 FragColor;

in vec3 Normal;
in vec2 TexCoords;
in vec3 Color;

void main()
{
	FragColor = vec4(Color, 1.0);
}