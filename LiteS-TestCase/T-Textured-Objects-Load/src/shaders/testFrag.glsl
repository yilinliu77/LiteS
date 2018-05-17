    #version 330 core

    out vec4 FragColor;

    //layout (location = 0) out vec4 F1;
    //layout (location = 1) out vec4 F2;

    struct Material {
        vec3 diffuse;
        vec3 specular;
        float shininess;
    }; 

    struct DirLight {
        vec3 direction;
        vec3 color;
    };

    in vec3 FragPos;
    in vec3 Normal;
    in vec2 TexCoords;

    uniform vec3 viewPos;
    uniform Material material;
    uniform sampler2D texture_diffuse1;

    uniform bool useTexture;

    DirLight dirLight;

    // function prototypes
    vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);

    void main()
    {    
        vec3 final=vec3(0,0,0);
        if(useTexture){
            final += texture(texture_diffuse1, TexCoords).rgb;
        }
        else{
            dirLight.direction=vec3(-1,-1,-1);
            dirLight.color=vec3(1.0,1.0,1.0);
            // properties
            vec3 norm = normalize(Normal);
            vec3 viewDir = normalize(viewPos - FragPos);

            vec3 result = CalcDirLight(dirLight, norm, viewDir);  
            
            final+=result;
        }
        
        FragColor = vec4(final, 1.0);
        //FragColor = vec4(0.0,1.0,0.0, 1.0);
        //F1=vec4(0,0.5,0, 1.0);
        //F2=vec4(1,0.5,1, 1.0);
        
    }

    // calculates the color when using a directional light.
    vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir)
    {
        vec3 lightDir = normalize(-light.direction);
        // diffuse shading
        float diff = max(dot(normal, lightDir), 0.0);
        // specular shading
        vec3 reflectDir = reflect(-lightDir, normal);
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
        // combine results
        vec3 ambient = vec3(0.1,0.1,0.1);
        vec3 diffuse = light.color * diff * material.diffuse;
        vec3 specular = light.color * spec * material.specular;
        //return (ambient + diffuse + specular);
        return (ambient + diffuse);    
    }