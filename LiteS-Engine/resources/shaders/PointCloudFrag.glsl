#version 330 core

out vec4 FragColor;

in VS_OUT {
    vec3 Normal;
    vec3 Color;  
} gs_in;

in vec3 fColor;

void main()
{
  FragColor = vec4(gs_in.Color, 1.0);
}