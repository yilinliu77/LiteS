#version 330 core
out vec4 FragColor;
in vec2 TexCoords;

uniform sampler2D gAlbedo;
uniform sampler2D ssao;

void main()
{             
    // Retrieve data from g-buffer
    vec3 Diffuse = texture(gAlbedo, TexCoords).rgb;
    float AmbientOcclusion = texture(ssao, TexCoords).r;
    
	FragColor = vec4(AmbientOcclusion*Diffuse, 1.0);
}