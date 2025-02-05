struct Input
{
    float4 Position : SV_Position;
    float3 Normal : TEXCOORD0;
};

float3 GetColorFromPositionAndNormal( in float3 worldPosition, in float3 normal ) {
    const float pi = 3.141519;

    float3 scaledPos = worldPosition.xyz * pi * 2.0;
    float3 scaledPos2 = worldPosition.xyz * pi * 2.0 / 10.0 + float3( pi / 4.0 );
    float s = cos( scaledPos2.x ) * cos( scaledPos2.y ) * cos( scaledPos2.z );  // [-1,1] range
    float t = cos( scaledPos.x ) * cos( scaledPos.y ) * cos( scaledPos.z );     // [-1,1] range

    float3 colorMultiplier = float3( 0.5, 0.5, 1 );
    if ( abs( normal.x ) > abs( normal.y ) && abs( normal.x ) > abs( normal.z ) ) {
        colorMultiplier = float3( 1, 0.5, 0.5 );
    } else if ( abs( normal.y ) > abs( normal.x ) && abs( normal.y ) > abs( normal.z ) ) {
        colorMultiplier = float3( 0.5, 1, 0.5 );
    }

    t = ceil( t * 0.9 );
    s = ( ceil( s * 0.9 ) + 3.0 ) * 0.25;
    float3 colorB = float3( 0.85, 0.85, 0.85 );
    float3 colorA = float3( 1, 1, 1 );
    float3 finalColor = lerp( colorA, colorB, t ) * s;

    return colorMultiplier * finalColor;
}

float4 main(Input input) : SV_Target0
{
    float dx = 0.25;
    float dy = 0.25;
    float3 colorMultiplier = float3( 0.0, 0.0, 0.0 );
    for ( float y = 0.0; y < 1.0; y += dy ) {
        for ( float x = 0.0; x < 1.0; x += dx ) {
            float4 samplePos = float4(input.Position.xyz + ddx(input.Position.xyz) * x + ddy(input.Position.xyz) * y, 1.0);
            colorMultiplier += GetColorFromPositionAndNormal(samplePos.xyz, input.Normal) * dx * dy;
        }
    }

    float4 finalColor;
    finalColor.rgb = colorMultiplier;
    finalColor.a = 1.0;
    return finalColor;
}
