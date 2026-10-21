
#type vertex
#version 330 core

// Mesh는 삼각형 정점과 연결 번호를 모은 형상이다. 정점에서 위치와 표면에 수직인 방향(법선)을 읽는다.

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_Normal;
out vec3 v_WorldPosition;

void main()
{
    // Local 좌표는 Mesh의 기준점에서 잰 위치이고 World 좌표는 장면 전체 기준 위치다. 모델 행렬로 Local을 World로 바꿔 표면과 광원을 같은 기준에서 비교한다.
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;

    // 크기 배율이 축마다 다르면 방향만 모델 행렬로 바꾼 법선은 표면에 수직이 아니게 된다.
    // 역전치 행렬을 적용해 변환 뒤에도 표면에 수직인 방향을 유지한다.
    v_Normal = mat3(transpose(inverse(u_Model))) * a_Normal;


    // 좌표 변환은 점의 기준을 바꾸는 계산이다. Local에서 World, Camera, Clip 순서로 바꾸며 Camera 좌표는 카메라가 원점인 기준이고 Clip 좌표는 화면 자르기와 깊이 계산에 쓰이는 중간값이다.
    // Projection 원근 계산으로 먼 표면은 작아 보이고 Clip 좌표를 w로 나눈 뒤 GPU가 화면 위치와 앞뒤 깊이를 정한다.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core


in vec3 v_Normal;
in vec3 v_WorldPosition;

uniform vec3 u_CameraPosition;
uniform vec3 u_LightDirection;

uniform vec4 u_BaseColorFactor;
uniform float u_MetallicFactor;
uniform float u_RoughnessFactor;


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

void main()
{
    vec3 N = normalize(v_Normal);
    vec3 V = normalize(u_CameraPosition - v_WorldPosition);
    vec3 L = normalize(u_LightDirection);

    float NdotL = max(dot(N, L), 0.0);

    float upFactor = N.y * 0.5 + 0.5;

    vec3 groundAmbient = vec3(0.10, 0.105, 0.11);
    vec3 skyAmbient = vec3(0.24, 0.25, 0.27);
    vec3 ambientLight = mix(groundAmbient, skyAmbient, upFactor);

    vec3 baseColor = u_BaseColorFactor.rgb;

    float baseAlpha = u_BaseColorFactor.a;

    // 재질의 금속성과 거칠기를 제한해 단순한 빛 반사 계산에 사용한다.
    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    // 금속 계수가 커져도 표면 색이 모두 사라지지 않도록 기본색 조명을 최대 35% 남기는 단순 근사다.
    vec3 diffuseColor = baseColor * mix(1.0, 0.35, metallic);

    // 표면에서 빛과 카메라를 향하는 두 방향의 중간을 이용해 반짝임을 계산한다. 거칠수록 넓고 약해진다.
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);

    float shininess = mix(48.0, 5.0, roughness);
    float specularStrength = pow(NdotH, shininess);

    if (NdotL <= 0.0)
        specularStrength = 0.0;

    // 표면을 정면에서 볼 때의 반사색을 비금속의 회색 값 0.04와 금속의 기본색 사이에서 정한다.
    vec3 F0 = mix(vec3(0.04), baseColor, metallic);
    vec3 specular = F0 * specularStrength * 0.35;

    vec3 directLight = (diffuseColor * NdotL + specular) * 1.35;

    vec3 fillDirection = normalize(vec3(0.65, 0.35, -0.65));
    float fillAmount = max(dot(N, fillDirection), 0.0);
    vec3 fillLight = diffuseColor * fillAmount * 0.42;

    vec3 color =
        baseColor * ambientLight +
        directLight +
        fillLight;

    // ACES 곡선은 조명 결과를 화면 밝기 범위로 압축한다. 이어 1/2.2 거듭제곱 감마 보정으로 모니터 표시용 RGB 값에 맞춘다.
    color = ToneMapACES(color);

    color = pow(color, vec3(1.0 / 2.2));

    FragColor =vec4(color,baseAlpha);
}
