#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoords;
layout(location = 3) in vec3 aColor;

out VS_OUT {
	vec3 Normal;
	vec3 Color;
} vs_out;

uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;

void main()
{
	gl_Position = projection * view * model * vec4(aPos, 1.0f);
	vs_out.Normal = normalize(mat3(transpose(inverse(view * model))) * aNormal);
	vs_out.Color = aColor;
}