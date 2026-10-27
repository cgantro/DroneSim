
#type vertex
#version 330 core

// Mesh는 삼각형 정점과 연결 번호를 모은 형상이다. 정점에서 위치와 표면에 수직인 방향(법선)을 읽는다.

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;
layout(location = 2) in vec2 a_TextureCoordinate;
layout(location = 3) in vec4 a_InstanceColumn0;
layout(location = 4) in vec4 a_InstanceColumn1;
layout(location = 5) in vec4 a_InstanceColumn2;
layout(location = 6) in vec4 a_InstanceColumn3;

uniform mat4 u_Model;
uniform int u_UseInstances;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_Normal;
out vec3 v_ModelNormal;
out vec3 v_ModelPosition;
out vec3 v_WorldPosition;
out vec2 v_TextureCoordinate;

void main()
{
    mat4 model = u_UseInstances != 0
        ? mat4(a_InstanceColumn0, a_InstanceColumn1, a_InstanceColumn2, a_InstanceColumn3)
        : u_Model;
    // Local 좌표는 Mesh의 기준점에서 잰 위치이고 World 좌표는 장면 전체 기준 위치다. 모델 행렬로 Local을 World로 바꿔 표면과 광원을 같은 기준에서 비교한다.
    vec4 worldPosition = model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;
    v_ModelPosition = a_Position;
    v_ModelNormal = a_Normal;
    v_TextureCoordinate = a_TextureCoordinate;

    // 크기 배율이 축마다 다르면 방향만 모델 행렬로 바꾼 법선은 표면에 수직이 아니게 된다.
    // 역전치 행렬을 적용해 변환 뒤에도 표면에 수직인 방향을 유지한다.
    v_Normal = mat3(transpose(inverse(model))) * a_Normal;


    // 좌표 변환은 점의 기준을 바꾸는 계산이다. Local에서 World, Camera, Clip 순서로 바꾸며 Camera 좌표는 카메라가 원점인 기준이고 Clip 좌표는 화면 자르기와 깊이 계산에 쓰이는 중간값이다.
    // Projection 원근 계산으로 먼 표면은 작아 보이고 Clip 좌표를 w로 나눈 뒤 GPU가 화면 위치와 앞뒤 깊이를 정한다.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core
in vec3 v_Normal;
in vec3 v_ModelNormal;
in vec3 v_ModelPosition;
in vec3 v_WorldPosition;
in vec2 v_TextureCoordinate;

uniform vec3 u_CameraPosition;
uniform vec3 u_LightDirection;

uniform vec4 u_BaseColorFactor;
uniform float u_MetallicFactor;
uniform float u_RoughnessFactor;
uniform sampler2D u_BaseTexture;
uniform int u_UseBaseTexture;
uniform int u_TriplanarTexture;
uniform int u_TerrainSurface;
uniform int u_FlowingWater;
uniform float u_TimeSeconds;
out vec4 FragColor;

vec3 ToneMapACES(vec3 color)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;

    // 여러 조명을 더해 1보다 커진 선형 RGB를 화면에 표시할 수 있는 0~1 범위로 부드럽게 줄인다.
    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0
    );
}

float DistributionGGX(vec3 normal, vec3 halfway, float roughness)
{
    const float pi = 3.14159265;
    float alpha = roughness * roughness;
    float alphaSquared = alpha * alpha;
    float normalHalfway = max(dot(normal, halfway), 0.0);
    float denominator = normalHalfway * normalHalfway * (alphaSquared - 1.0) + 1.0;
    return alphaSquared / max(pi * denominator * denominator, 0.0001);
}

float GeometrySchlickGGX(float normalView, float roughness)
{
    float radius = roughness + 1.0;
    float k = radius * radius / 8.0;
    return normalView / max(normalView * (1.0 - k) + k, 0.0001);
}

vec3 FresnelSchlick(float viewHalfway, vec3 reflectionAtNormal)
{
    return reflectionAtNormal + (1.0 - reflectionAtNormal)
        * pow(1.0 - clamp(viewHalfway, 0.0, 1.0), 5.0);
}

vec3 SampleTriplanarTexture(vec3 position, vec3 normal)
{
    // UV가 없는 X500 메시에도 좌표축 투영을 겹쳐 적용해 표면 결을 입힌다.
    vec3 weights = pow(abs(normal), vec3(4.0));
    weights /= max(weights.x + weights.y + weights.z, 0.0001);
    vec2 scale = vec2(12.0);
    vec3 alongX = texture(u_BaseTexture, fract(position.zy * scale)).rgb;
    vec3 alongY = texture(u_BaseTexture, fract(position.xz * scale)).rgb;
    vec3 alongZ = texture(u_BaseTexture, fract(position.xy * scale)).rgb;
    return alongX * weights.x + alongY * weights.y + alongZ * weights.z;
}

void main()
{
    vec3 N = normalize(v_Normal);
    vec3 V = normalize(u_CameraPosition - v_WorldPosition);
    vec3 L = normalize(u_LightDirection);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0);

    float upFactor = N.y * 0.5 + 0.5;

    vec3 baseColor = u_BaseColorFactor.rgb;
    if (u_UseBaseTexture != 0)
    {
        vec3 surfaceColor = u_TriplanarTexture != 0
            ? SampleTriplanarTexture(v_ModelPosition, normalize(v_ModelNormal))
            : texture(u_BaseTexture, v_TextureCoordinate).rgb;
        baseColor *= surfaceColor;
    }

    if (u_FlowingWater != 0)
    {
        // 장면 좌표를 따라 흐르는 밝은 줄무늬로 강물의 이동을 보여준다.
        vec2 flowPosition = v_ModelPosition.xz;
        float current = sin(flowPosition.x * 0.19 + flowPosition.y * 0.07
            - u_TimeSeconds * 1.8);
        float ripple = sin(flowPosition.x * 0.48 - flowPosition.y * 0.16
            - u_TimeSeconds * 3.2);
        float foam = smoothstep(0.72, 0.96, current * 0.72 + ripple * 0.28);
        baseColor = mix(baseColor, vec3(0.36, 0.69, 0.77), foam * 0.24);
    }

    float baseAlpha = u_BaseColorFactor.a;

    // 재질의 금속성과 거칠기를 제한해 단순한 빛 반사 계산에 사용한다.
    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    vec3 H = normalize(L + V);
    vec3 F0 = mix(vec3(0.04), baseColor, metallic);
    vec3 fresnel = FresnelSchlick(max(dot(H, V), 0.0), F0);
    float distribution = DistributionGGX(N, H, roughness);
    float geometry = GeometrySchlickGGX(NdotV, roughness)
        * GeometrySchlickGGX(NdotL, roughness);
    vec3 specular = distribution * geometry * fresnel
        / max(4.0 * NdotV * NdotL, 0.001);
    vec3 diffuse = (1.0 - fresnel) * (1.0 - metallic) * baseColor / 3.14159265;
    vec3 directLight = (diffuse + specular) * NdotL * 2.2;

    vec3 fillDirection = normalize(vec3(0.65, 0.35, -0.65));
    float fillAmount = max(dot(N, fillDirection), 0.0);
    vec3 groundAmbient = vec3(0.10, 0.105, 0.11);
    vec3 skyAmbient = vec3(0.24, 0.25, 0.27);
    vec3 ambientLight = mix(groundAmbient, skyAmbient, upFactor);
    vec3 fillLight = baseColor * fillAmount * (1.0 - metallic) * 0.20;

    // 정사영상에는 빛과 그림자가 이미 담겨 있어, 지면은 사진 색을 그대로 쓴다.
    vec3 color = u_TerrainSurface != 0
        ? baseColor
        : baseColor * ambientLight * (1.0 - metallic) + directLight + fillLight;

    // 먼 산은 대기 색과 섞어 지형의 크기와 깊이가 눈에 들어오게 한다.
    float viewDistance = length(u_CameraPosition - v_WorldPosition);
    float fogAmount = smoothstep(800.0, 4200.0, viewDistance);
    color = mix(color, vec3(0.43, 0.60, 0.72), fogAmount);

    // ACES 곡선은 조명 결과를 화면 밝기 범위로 압축한다. 이어 1/2.2 거듭제곱 감마 보정으로 모니터 표시용 RGB 값에 맞춘다.
    if (u_TerrainSurface == 0)
        color = ToneMapACES(color);

    color = pow(color, vec3(1.0 / 2.2));

    FragColor = vec4(color, baseAlpha);
}
