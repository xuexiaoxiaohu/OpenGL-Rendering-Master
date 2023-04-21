#version 450 core

struct material{
    vec3 ambient,diffuse,specular;    
    float shininess;
}; 

struct dirLight{
    vec3 direction,ambient,diffuse,specular;
};

in vec3 vPosition;
in vec3 vNormal;  

uniform vec3 viewPos;
uniform material mat;
uniform dirLight dl1, dl2;

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir);

void main(){
    vec3 norm = normalize(vNormal);
    vec3 viewDir = normalize(viewPos - vPosition);

    vec3 result = calcDirLight(dl1, norm, viewDir);
    result += calcDirLight(dl2, norm, viewDir);

    gl_FragColor = vec4(result, 1.0);
} 

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir){
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), mat.shininess);
    vec3 ambient  = light.ambient * mat.ambient;
    vec3 diffuse  = light.diffuse * (diff * mat.diffuse);
    vec3 specular = light.specular * (spec * mat.specular); 
    return (ambient + diffuse + specular);
}