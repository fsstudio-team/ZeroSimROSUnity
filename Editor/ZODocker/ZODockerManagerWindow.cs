using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO;
using System.Text;
using System.Diagnostics;
using MS.Shell.Editor;
using System.Collections.Generic;

namespace ZO.Editor {

public class ZODockerManagerWindow : EditorWindow {
    private static ZODockerManagerWindow _window = null;

    private string _dockerLogColor = "#207020";

    [MenuItem("Zero Sim/Docker Manager")]
    public static void OpenDockerManager() {
        
        if(_window == null){
            
            _window = GetWindow(
                typeof(ZODockerManagerWindow), 
                utility:false,  
                title: "ZO Docker Manager", 
                focus: true) as ZODockerManagerWindow;
        }

        _window.ShowUtility();
    }

    async void Awake()
    {
        bool isDockerRunning = await ZODockerManager.IsZODockerRunning();
        UnityEngine.Debug.Log($"Docker running on awake: {isDockerRunning}");
    }

    private void OnGUI() {
        
        EditorGUILayout.LabelField("Docker running", ZODockerManager.isRunning.ToString());

        if(!ZODockerManager.isRunning && GUILayout.Button("Select docker-compose.yml working directory")){
            ZODockerManager.composeWorkingDirectory = EditorUtility.OpenFolderPanel("Select docker-compose.yml directory", "", "");
        }
        EditorGUILayout.LabelField("Docker-compose directory", ZODockerManager.composeWorkingDirectory);

        if (!ZODockerManager.isRunning && GUILayout.Button("Start Docker service")){
            ZODockerManager.DockerComposeUp();
        }

        if (ZODockerManager.isRunning && GUILayout.Button("Stop Docker service")){
            ZODockerManager.DockerComposeDown();
        }

        ZODockerManager.showLogs = EditorGUILayout.Toggle("Show docker-compose logs", ZODockerManager.showLogs);
        
    }

    
}

}