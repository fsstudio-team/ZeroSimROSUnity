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

    private float _dockerPollTimespan = 2.0f;
    private double _lastPollTimestamp = 0;

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

    async void Awake(){
        CheckDockerInstallation();

        _lastPollTimestamp = EditorApplication.timeSinceStartup;
    }

    private async void CheckDockerInstallation(){
        bool isDockerInstalled = await ZODockerManager.IsZODockerInstalled();

        if(isDockerInstalled){
            bool isDockerRunning = await ZODockerManager.IsZODockerRunning();
            UnityEngine.Debug.Log($"Docker running: {isDockerRunning}");
        }
    }

    private void OnGUI() {
        
        if(ZODockerManager.isInstalled)
            OnGUIDocker();
        else
            OnGUIInstallDocker();
        
    }

    private void OnGUIDocker(){
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

    private async void OnGUIInstallDocker(){

        EditorGUILayout.LabelField("Docker installation not found. We're checking for a new installation in the background...");
        
        if(EditorApplication.timeSinceStartup - _lastPollTimestamp > _dockerPollTimespan){
            CheckDockerInstallation();
            _lastPollTimestamp = EditorApplication.timeSinceStartup;
        }
        
    }
}

}