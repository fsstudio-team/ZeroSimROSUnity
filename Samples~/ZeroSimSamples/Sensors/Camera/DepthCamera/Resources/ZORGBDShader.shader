Shader "ZeroSim/ZORGBDShader"
{
    Properties
    {
        [HideInInspector]_MainTex ("Texture", 2D) = "white" {}

        [Toggle(FLIP_Y)]
        _FlipY ("Flip Y", Float) = 0        

        [Toggle(FLIP_X)]
        _FlipX ("Flip X", Float) = 0        

    }
    SubShader
    {
        // markers that specify that we don't need culling 
        // or comparing/writing to the depth buffer
        Cull Off
        ZWrite Off 
        ZTest Always

        Pass
        {
            Tags{ "RenderType" = "Opaque" }
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma shader_feature FLIP_Y
            #pragma shader_feature FLIP_X


            // make fog work
            #pragma multi_compile_fog

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
                float4 viewDir : TEXCOORD1;
            };

            sampler2D _MainTex;

            /// Depth textures are available for sampling in shaders as global shader properties. 
            /// By declaring a sampler called _CameraDepthTexture you will be able to sample the main 
            /// depth texture for the camera.
            /// See: https://docs.unity3d.com/Manual/SL-CameraDepthTexture.html
            sampler2D _CameraDepthTexture;

            v2f vert (appdata v)
            {
                v2f o;
                //convert the vertex positions from object space to clip space so they can be rendered
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                #ifdef FLIP_Y                
                    o.uv.y = 1.0 - o.uv.y;
                #endif       

                #ifdef FLIP_X                
                    o.uv.x = 1.0 - o.uv.x;
                #endif 

                o.viewDir = mul (unity_CameraInvProjection, float4 (o.uv * 2.0 - 1.0, 1.0, 1.0));               

                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);
                //get depth from depth texture
                float depth01 = tex2D(_CameraDepthTexture, i.uv).r;
                // //linear depth between camera and far clipping plane
                depth01 = Linear01Depth(depth01);
                //depth as distance from camera in units 
                // depth = depth * _ProjectionParams.z;
                float3 viewPos = (i.viewDir.xyz / i.viewDir.w) * depth01;
                col.a = length(viewPos);
                return col;
            }
            ENDCG
        }
    }
}
