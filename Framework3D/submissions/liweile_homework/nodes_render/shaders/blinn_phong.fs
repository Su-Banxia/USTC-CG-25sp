#version 430 core

// Define a uniform struct for lights
struct Light {
    // The matrices are used for shadow mapping. You need to fill it according to how we are filling it when building the normal maps (node_render_shadow_mapping.cpp). 
    // Now, they are filled with identity matrix. You need to modify C++ code innode_render_deferred_lighting.cpp.
    // Position and color are filled.
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color; // Just use the same diffuse and specular color.
    int shadow_map_id;
};

layout(binding = 0) buffer lightsBuffer {
Light lights[4];
};

uniform vec2 iResolution;   // width and height of the screen

uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // You should apply normal mapping in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

// uniform float alpha;
uniform vec3 camPos;

uniform int light_count;

layout(location = 0) out vec4 Color;

float GetShadow(vec4 lightSpacePos, int lightIndex, float diff) 
{
    float bias = max(0.05 * (1.0 - diff), 0.005);
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    float currentDepth = projCoords.z; 
    projCoords = projCoords * 0.5 + 0.5;
    
    float closestDepth = texture(shadow_maps, vec3(projCoords.xy, lightIndex)).r;
    
    return currentDepth - bias > closestDepth ? 1.0 : 0.0;
}

void main() 
{
    vec2 uv = gl_FragCoord.xy / iResolution;

    vec3 pos = texture2D(position,uv).xyz;
    vec3 normal = texture2D(normalMapSampler,uv).xyz;

    vec4 metalnessRoughness = texture2D(metallicRoughnessSampler,uv);
    float metal = metalnessRoughness.x;
    float roughness = metalnessRoughness.y;
    Color = vec4(0,0,0,1);

    float k_s = metal * 0.8;
    float k_d = 1.0 - k_s;
    float k_a = 0.3;

    // ambient part
    vec3 albedo = texture(diffuseColorSampler, uv).rgb;
    vec3 ambient = k_a * albedo;

    Color.rgb += ambient;

    for(int i = 0; i < light_count; i++) 
    {
        Light light = lights[i];

        float shadow_map_value = texture(shadow_maps, vec3(uv, light.shadow_map_id)).x;

        // diffuse part
        vec3 norm = normalize(normal);
        vec3 lightDir = normalize(light.position - pos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = k_d * light.color * (diff * albedo);

        // specular part
        float alpha = (1.0 - roughness) * 150;
        vec3 viewDir = normalize(camPos - pos);
        vec3 halfDir = normalize(lightDir + viewDir);
        float spec = 0.0;
        if (abs(diff) > 1e-10)
        {
            spec = pow( max(dot(norm, halfDir), 0.0), alpha );
        }
        vec3 specular = k_s * spec * light.color;

        // shadow part
        vec4 lightSpacePos = light.light_projection * light.light_view * vec4(pos, 1.0);
        float shadow = GetShadow(lightSpacePos, light.shadow_map_id, diff);

        // if (shadow > 0.5) 
        // {
        //     Color.rgb = vec3(1,0,0); // Red for shadow area
        // } 
        // else 
        // {
        //     Color.rgb = vec3(0,1,0); // Green for not shadow are
        // }

        vec3 lighting = (diffuse + specular) * (1.0 - 0.90 * shadow);
        Color.rgb += lighting;
    }

}