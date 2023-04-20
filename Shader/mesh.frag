#version 450 core

#define NUM 4

struct material{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 

struct dirLight{
    vec3 direction;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 worldPos;  
in vec3 normal;  
in vec3 vPosition;

out vec4 fragColor;

uniform vec3 viewPos;
uniform material mat;
uniform dirLight dl1;
uniform dirLight dl2;

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir);

void main(){
    vec3 norm = normalize(normal);
    vec3 viewDir = normalize(viewPos - vPosition);

    vec3 result = calcDirLight(dl1, norm, viewDir);
    result += calcDirLight(dl2, norm, viewDir);

    fragColor = vec4(result, 1.0);
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