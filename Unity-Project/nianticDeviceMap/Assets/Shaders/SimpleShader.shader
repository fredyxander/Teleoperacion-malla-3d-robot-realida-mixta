Shader "Lightship/SimpleShader"
{
    Properties
    {
        _AlphaAmount("Alpha Amount (Transparency)", Range(0, 1)) = 0.025
        _TintColor("Tint Color", Color) = (0.85, 0.9, 1.0, 1.0)
    }

        SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }
        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Back

        Pass
        {
            Name "FlatTransparent"
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
            };

            float _AlphaAmount;
            fixed4 _TintColor;

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                // Color uniforme translúcido
                return fixed4(_TintColor.rgb, _AlphaAmount);
            }
            ENDCG
        }
    }

        Fallback "Unlit/Color"
}
