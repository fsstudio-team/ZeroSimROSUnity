using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using System.Linq;

namespace ZO.ImportExport {
    public class ZOImportOBJ {
        public static string WorkingDirectory {
            get; set;
        } = ".";

        public static string FileName {
            get; set;
        } = "temp";


        struct OBJFace {
            public string materialName;
            public string meshName;
            public int[] indexes;
        }

        protected static Vector2 ParseVector2(string[] lineComponents) {
            Vector2 result = new Vector2(float.Parse(lineComponents[1]), float.Parse(lineComponents[2]));
            return result;
        }

        protected static Vector3 ParseVector3(string[] lineComponents) {
            Vector3 result = new Vector3(float.Parse(lineComponents[1]), float.Parse(lineComponents[2]), float.Parse(lineComponents[3]));
            return result;
        }

        protected static Color ParseColor(string[] lineComponents, float scale = 1.0f) {
            Color result = new Color(float.Parse(lineComponents[1]) * scale, float.Parse(lineComponents[2]) * scale, float.Parse(lineComponents[3]) * scale);
            return result;
        }

        protected static Material[] ImportMTLFile(string mtlFilePath) {
            List<Material> materials = new List<Material>();
            Material currentMaterial = null;

            string workingDirectory = Path.GetDirectoryName(mtlFilePath);

            using (StreamReader streamReader = new StreamReader(mtlFilePath)) {
                while (!streamReader.EndOfStream) {
                    string ln = streamReader.ReadLine();
                    string l = ln.Trim().Replace("  ", " ");
                    string[] lineComponents = l.Split(' ');
                    string data = l.Remove(0, l.IndexOf(' ') + 1);

                    if (lineComponents[0] == "newmtl") {
                        if (currentMaterial != null) {
                            materials.Add(currentMaterial);
                        }
                        currentMaterial = new Material(Shader.Find("Standard (Specular setup)"));
                        currentMaterial.name = data;
                    } else if (lineComponents[0] == "Kd") {
                        currentMaterial.SetColor("_Color", ParseColor(lineComponents));
                    } else if (lineComponents[0] == "map_Kd") {
                        string texturePath = Path.Combine(workingDirectory, lineComponents[1]);
                        string textureType = Path.GetExtension(texturePath);
                        if (textureType == ".png" || textureType == ".jpg") {
                            Texture2D texture = new Texture2D(1,1);
                            texture.LoadImage(File.ReadAllBytes(texturePath));
                            currentMaterial.SetTexture("_MainTex", texture);
                        }
                        
                    } else if (lineComponents[0] == "map_Bump") {
                        // TODO
                    } else if (lineComponents[0] == "Ks") {
                        currentMaterial.SetColor("_SpecColor", ParseColor(lineComponents));
                    } else if (lineComponents[0] == "Ka") {
                        currentMaterial.SetColor("_EmissionColor", ParseColor(lineComponents, 0.05f));
                        currentMaterial.EnableKeyword("_EMISSION");
                    } else if (lineComponents[0] == "d") {
                        float visibility = float.Parse(lineComponents[1]);
                        if (visibility < 1) {
                            Color temp = currentMaterial.color;

                            temp.a = visibility;
                            currentMaterial.SetColor("_Color", temp);

                            // set transparency
                            currentMaterial.SetFloat("_Mode", 3);
                            currentMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                            currentMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                            currentMaterial.SetInt("_ZWrite", 0);
                            currentMaterial.DisableKeyword("_ALPHATEST_ON");
                            currentMaterial.EnableKeyword("_ALPHABLEND_ON");
                            currentMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                            currentMaterial.renderQueue = 3000;
                        }

                    } else if (lineComponents[0] == "Ns") {
                        float Ns = float.Parse(lineComponents[1]);
                        Ns = (Ns / 1000);
                        currentMaterial.SetFloat("_Glossiness", Ns);

                    }
                }
            }

            if (currentMaterial != null) {
                materials.Add(currentMaterial);
            }

            return materials.ToArray();
        }

        public static GameObject Import(string objFilePath) {
            using (StreamReader streamReader = new StreamReader(objFilePath)) {
                return Import(streamReader, Path.GetFileNameWithoutExtension(objFilePath), Path.GetDirectoryName(objFilePath));
            }
        }

        public static GameObject Import(StreamReader streamReader, string meshName, string workingDirectory) {


            bool hasNormals = false;
            List<Vector3> vertices = new List<Vector3>();
            List<Vector3> normals = new List<Vector3>();
            List<Vector2> uvs = new List<Vector2>();

            List<Vector3> uvertices = new List<Vector3>();
            List<Vector3> unormals = new List<Vector3>();
            List<Vector2> uuvs = new List<Vector2>();

            List<string> materialNames = new List<string>();
            List<string> objectNames = new List<string>();
            Dictionary<string, int> hashtable = new Dictionary<string, int>();
            List<OBJFace> faceList = new List<OBJFace>();
            string cmaterial = "";
            string cmesh = "default";
            //CACHE
            Material[] materials = null;

            while (!streamReader.EndOfStream) {
                string ln = streamReader.ReadLine();
                if (ln.Length > 0 && ln[0] != '#') {
                    string l = ln.Trim().Replace("  ", " ");
                    string[] lineComponents = l.Split(' ');
                    string data = l.Remove(0, l.IndexOf(' ') + 1);

                    if (lineComponents[0] == "mtllib") {
                        string mtlFilePath = Path.Combine(workingDirectory, lineComponents[1]);
                        materials = ImportMTLFile(mtlFilePath);

                    } else if ((lineComponents[0] == "g" || lineComponents[0] == "o")) {
                        cmesh = data;
                        if (!objectNames.Contains(cmesh)) {
                            objectNames.Add(cmesh);
                        }
                    } else if (lineComponents[0] == "usemtl") {
                        cmaterial = data;
                        if (!materialNames.Contains(cmaterial)) {
                            materialNames.Add(cmaterial);
                        }

                    } else if (lineComponents[0] == "v") {
                        //VERTEX
                        vertices.Add(ParseVector3(lineComponents));
                    } else if (lineComponents[0] == "vn") {
                        //VERTEX NORMAL
                        normals.Add(ParseVector3(lineComponents));
                    } else if (lineComponents[0] == "vt") {
                        //VERTEX UV
                        uvs.Add(ParseVector2(lineComponents));
                    } else if (lineComponents[0] == "f") {
                        int[] indexes = new int[lineComponents.Length - 1];
                        for (int i = 1; i < lineComponents.Length; i++) {
                            string component = lineComponents[i];
                            int vertexIndex = -1;
                            int normalIndex = -1;
                            int uvIndex = -1;
                            if (component.Contains("//")) {
                                //doubleslash, no UVS.
                                string[] elementComps = component.Split('/');
                                vertexIndex = int.Parse(elementComps[0]) - 1;
                                normalIndex = int.Parse(elementComps[2]) - 1;
                            } else if (component.Count(x => x == '/') == 2) {
                                //contains everything
                                string[] elementComps = component.Split('/');
                                vertexIndex = int.Parse(elementComps[0]) - 1;
                                uvIndex = int.Parse(elementComps[1]) - 1;
                                normalIndex = int.Parse(elementComps[2]) - 1;
                            } else if (!component.Contains("/")) {
                                //just vertex inedx
                                vertexIndex = int.Parse(component) - 1;
                            } else {
                                //vertex and uv
                                string[] elementComps = component.Split('/');
                                vertexIndex = int.Parse(elementComps[0]) - 1;
                                uvIndex = int.Parse(elementComps[1]) - 1;
                            }
                            string hashEntry = vertexIndex + "|" + normalIndex + "|" + uvIndex;
                            if (hashtable.ContainsKey(hashEntry)) {
                                indexes[i - 1] = hashtable[hashEntry];
                            } else {
                                //create a new hash entry
                                indexes[i - 1] = hashtable.Count;
                                hashtable[hashEntry] = hashtable.Count;
                                uvertices.Add(vertices[vertexIndex]);
                                if (normalIndex < 0 || (normalIndex > (normals.Count - 1))) {
                                    unormals.Add(Vector3.zero);
                                } else {
                                    hasNormals = true;
                                    unormals.Add(normals[normalIndex]);
                                }
                                if (uvIndex < 0 || (uvIndex > (uvs.Count - 1))) {
                                    uuvs.Add(Vector2.zero);
                                } else {
                                    uuvs.Add(uvs[uvIndex]);
                                }

                            }
                        }
                        if (indexes.Length < 5 && indexes.Length >= 3) {
                            OBJFace f1 = new OBJFace();
                            f1.materialName = cmaterial;
                            f1.indexes = new int[] { indexes[0], indexes[1], indexes[2] };
                            f1.meshName = cmesh;
                            faceList.Add(f1);
                            if (indexes.Length > 3) {

                                OBJFace f2 = new OBJFace();
                                f2.materialName = cmaterial;
                                f2.meshName = cmesh;
                                f2.indexes = new int[] { indexes[2], indexes[3], indexes[0] };
                                faceList.Add(f2);
                            }
                        }
                    }
                }
            }

            if (objectNames.Count == 0)
                objectNames.Add("default");

            //build objects
            GameObject parentObject = new GameObject(meshName);


            foreach (string obj in objectNames) {
                GameObject subObject = new GameObject(obj);
                subObject.transform.parent = parentObject.transform;
                subObject.transform.localScale = new Vector3(-1, 1, 1);
                Mesh m = new Mesh();
                m.name = obj;
                List<Vector3> processedVertices = new List<Vector3>();
                List<Vector3> processedNormals = new List<Vector3>();
                List<Vector2> processedUVs = new List<Vector2>();
                List<int[]> processedIndexes = new List<int[]>();
                Dictionary<int, int> remapTable = new Dictionary<int, int>();
                List<string> meshMaterialNames = new List<string>();

                OBJFace[] objFaces = faceList.Where(x => x.meshName == obj).ToArray();
                foreach (string mn in materialNames) {
                    OBJFace[] faces = objFaces.Where(x => x.materialName == mn).ToArray();
                    if (faces.Length > 0) {
                        int[] indexes = new int[0];
                        foreach (OBJFace f in faces) {
                            int l = indexes.Length;
                            System.Array.Resize(ref indexes, l + f.indexes.Length);
                            System.Array.Copy(f.indexes, 0, indexes, l, f.indexes.Length);
                        }
                        meshMaterialNames.Add(mn);
                        if (m.subMeshCount != meshMaterialNames.Count)
                            m.subMeshCount = meshMaterialNames.Count;

                        for (int i = 0; i < indexes.Length; i++) {
                            int idx = indexes[i];
                            //build remap table
                            if (remapTable.ContainsKey(idx)) {
                                //ezpz
                                indexes[i] = remapTable[idx];
                            } else {
                                processedVertices.Add(uvertices[idx]);
                                processedNormals.Add(unormals[idx]);
                                processedUVs.Add(uuvs[idx]);
                                remapTable[idx] = processedVertices.Count - 1;
                                indexes[i] = remapTable[idx];
                            }
                        }

                        processedIndexes.Add(indexes);
                    } else {

                    }
                }

                m.vertices = processedVertices.ToArray();
                m.normals = processedNormals.ToArray();
                m.uv = processedUVs.ToArray();

                for (int i = 0; i < processedIndexes.Count; i++) {
                    m.SetTriangles(processedIndexes[i], i);
                }

                if (!hasNormals) {
                    m.RecalculateNormals();
                }
                m.RecalculateBounds();
                ;

                MeshFilter mf = subObject.AddComponent<MeshFilter>();
                MeshRenderer mr = subObject.AddComponent<MeshRenderer>();

                Material[] processedMaterials = new Material[meshMaterialNames.Count];
                for (int i = 0; i < meshMaterialNames.Count; i++) {

                    if (materials == null) {
                        processedMaterials[i] = new Material(Shader.Find("Standard (Specular setup)"));
                    } else {
                        Material mfn = Array.Find(materials, x => x.name == meshMaterialNames[i]); ;
                        if (mfn == null) {
                            processedMaterials[i] = new Material(Shader.Find("Standard (Specular setup)"));
                        } else {
                            processedMaterials[i] = mfn;
                        }

                    }
                    processedMaterials[i].name = meshMaterialNames[i];
                }

                mr.materials = processedMaterials;
                mf.mesh = m;

            }

            return parentObject;
        }

    }
}