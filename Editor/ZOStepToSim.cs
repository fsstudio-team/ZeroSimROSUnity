using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace ZO.Import {

    class ZOStepToSim : EditorWindow {

        static ZOStepToSim _window;

        Vector3 _meshScale;
        Vector3 _transformScale;
        float _linearDeflection;
        float _angularDeflection;
        bool _generateCollisionMeshes;
        string _stepFileDirectory = string.Empty;
        string _stepFilename = string.Empty;

        [MenuItem("Zero Sim/Import step file")]
        public static void OpenEditorWindow() {
            
            if(_window == null){
                
                _window = GetWindow(
                    typeof(ZOStepToSim), 
                    utility:false,  
                    title: "ZO Docker Manager", 
                    focus: true) as ZOStepToSim;
            }

            _window.ShowUtility();
        }

        public void ImportStepFile(){
            
            string arguments = 
                              $" --input_step_file=/input/{_stepFilename}"
                            + $" --output_directory /unity-project-root/Assets/"
                            + $" --mesh_scale {_meshScale.x} {_meshScale.y} {_meshScale.z}" 
                            + $" --transform_scale {_transformScale.x} {_transformScale.y} {_transformScale.z}" 
                            + " --linear_deflection=" + _linearDeflection.ToString()
                            + " --angular_deflection=" + _angularDeflection.ToString()
                            + (_generateCollisionMeshes ? " --generate_collision_meshes" : "");

            string command = "python /zo-asset-tools/zo_step_to_zosim/zo_step_to_zosim.py";
            string commandAWithArgs = $"{command} {arguments}";

            string[] additionalVolumes = { $"{_stepFileDirectory}:/input/" };

            ZO.Editor.ZODockerManager.DockerRun(service: "zosim_tools", commandAWithArgs, null, (exitCode) => {

                if(exitCode != 0){
                    UnityEngine.Debug.LogError($"Docker command error exit code: {exitCode}");
                    return;
                }

                UnityEngine.Debug.LogError($"Docker command exit code: {exitCode}");
            });
        }

        private void OnGUI() {
            EditorGUILayout.LabelField("Convex Decomposition Parameters:");
            _meshScale = EditorGUILayout.Vector3Field("Mesh scale:", _meshScale);
            _transformScale = EditorGUILayout.Vector3Field("Transform scale:", _transformScale);
            _linearDeflection = EditorGUILayout.FloatField("Linear deflection:", _linearDeflection);
            _angularDeflection = EditorGUILayout.FloatField("Angular deflection:", _angularDeflection);
            _generateCollisionMeshes = EditorGUILayout.Toggle("Generate collision meshes:", _generateCollisionMeshes);
            

            if(GUILayout.Button("Select step file")){
                string path = EditorUtility.OpenFilePanel("Select step file", "", "step");
                _stepFilename = System.IO.Path.GetFileName(path);
                _stepFileDirectory = System.IO.Path.GetDirectoryName(path);
            }

            EditorGUILayout.LabelField("Step file: ", _stepFilename);

            if(GUILayout.Button("Import step file")){
                ImportStepFile();
            }
        }

    }

}