#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoords;
layout(location = 3) in vec3 aColor;

out vec3 Normal;
out vec3 Color;

uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;

void main()
{
	gl_Position = projection * view * model * vec4(aPos, 1.0f);
	Normal = mat3(transpose(inverse(model))) * aNormal;
	Color = aColor;
}