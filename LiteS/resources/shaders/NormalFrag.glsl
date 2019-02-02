#version 330 core

out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3 viewPos;

void main()
{    
    vec3 final=vec3(1,0,0);
    
    FragColor = vec4(final, 1.0);
}