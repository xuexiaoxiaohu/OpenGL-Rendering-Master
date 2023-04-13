#version 450 core

#define NR_POINT_LIGHTS 4

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

struct pointLight{
    vec3 position;
    float constant;
    float linear;
    float quadratic;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};
struct spotLight{
    vec3 position;
    vec3 direction;
    float cutOff;
    float outerCutOff;
    float constant;
    float linear;
    float quadratic;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;       
};

in vec3 worldPos;  
in vec3 normal;  

out vec4 fragColor;

uniform vec3 viewPos;
uniform material mat;
uniform dirLight dl1;
uniform dirLight dl2;
uniform pointLight pls[NR_POINT_LIGHTS];
uniform spotLight sl;

vec3 calcDirLight(dirLight light, vec3 normal, vec3 viewDir);
vec3 calcPointLight(pointLight light, vec3 normal, vec3 worldPos, vec3 viewDir);
vec3 calcSpotLight(spotLight light, vec3 normal, vec3 worldPos, vec3 viewDir);

void main(){
    vec3 norm = normalize(normal);
    vec3 viewDir = normalize(viewPos - worldPos);

    vec3 result = calcDirLight(dl1, norm, viewDir);
    result += calcDirLight(dl2, norm, viewDir);

    //for(int i = 0; i < NR_POINT_LIGHTS; i++)
    //    result += CalcPointLight(pointLights[i], norm, FragPos, viewDir);    
    //result += CalcSpotLight(spotLight, norm, FragPos, viewDir);    

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
vec3 calcPointLight(pointLight light, vec3 normal, vec3 worldPos, vec3 viewDir){
    vec3 lightDir = normalize(light.position - worldPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), mat.shininess);
    float distance = length(light.position - worldPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance
    + light.quadratic * (distance * distance));    
    vec3 ambient = light.ambient * mat.diffuse;
    vec3 diffuse = light.diffuse * diff * mat.diffuse;
    vec3 specular = light.specular * spec * mat.specular;
    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    return (ambient + diffuse + specular);
}
vec3 calcSpotLight(spotLight light, vec3 normal, vec3 worldPos, vec3 viewDir){
    vec3 lightDir = normalize(light.position - worldPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), mat.shininess);
    float distance = length(light.position - worldPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance 
    + light.quadratic * (distance * distance));    
    float theta = dot(lightDir, normalize(-light.direction)); 
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    vec3 ambient = light.ambient * mat.diffuse;
    vec3 diffuse = light.diffuse * diff * mat.diffuse;
    vec3 specular = light.specular * spec * mat.specular;
    ambient *= attenuation * intensity;
    diffuse *= attenuation * intensity;
    specular *= attenuation * intensity;
    return (ambient + diffuse + specular);
}