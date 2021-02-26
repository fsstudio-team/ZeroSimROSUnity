Shader "ZOSim/ZOMonoPostProcessShader"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        [Toggle(FLIP_Y)]
        _FlipY ("Flip Y", Float) = 0        

        [Toggle(FLIP_X)]
        _FlipX ("Flip X", Float) = 0        

    }
    SubShader
    {
        // No culling or depth
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #pragma shader_feature FLIP_Y
            #pragma shader_feature FLIP_X

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                #ifdef FLIP_Y                
                    o.uv.y = 1.0 - o.uv.y;
                #endif       

                #ifdef FLIP_X                
                    o.uv.x = 1.0 - o.uv.x;
                #endif                

                return o;
            }

            sampler2D _MainTex;

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);
                float avg = (col.r * 0.21) + (col.g * 0.72) + (col.b * 0.07);
                col.r = avg;
                col.g = avg;
                col.b = avg;
                return col;
            }
            ENDCG
        }
    }
}
