using System;
using System.Collections;
using System.Collections.Generic;
using MS.Shell.Editor;
using UnityEngine;

namespace ZO.Editor{

public class ZODockerManager
{
    public static string dockerLogColor = "#207020";
    public static bool showLogs = true;
    public static bool isRunning = false;

    public static void DockerComposeUp(){
        // Navigate to parent directories where the docker and dockercompose files are located
        var options = new EditorShell.Options(){
            workDirectory = "../../docker/",
            //encoding = System.Text.Encoding.GetEncoding("GBK"),
            environmentVars = new Dictionary<string, string>(){
                //{"PATH", "usr/bin"}
            }
        };
        string command = "docker-compose up";
        // Execute docker command
        var task = EditorShell.Execute(command, options);
        DockerLog($"Starting docker container, please wait...");
        task.onLog += (EditorShell.LogType logType, string log) => {
            DockerLog(log);
        };
        task.onExit += (exitCode) => {
            DockerLog($"Docker compose up exit: {exitCode}", forceDisplay: true);
        };

        isRunning = true;
    }

    public static void DockerComposeDown(){
        // Navigate to parent directories where the docker and dockercompose files are located
        var options = new EditorShell.Options(){
            workDirectory = "../../docker/",
            //encoding = System.Text.Encoding.GetEncoding("GBK"),
            environmentVars = new Dictionary<string, string>(){
                //{"PATH", "usr/bin"}
            }
        };
        string command = "docker-compose down";
        // Execute docker command
        var task = EditorShell.Execute(command, options);
        DockerLog($"Stopping Docker, please wait...", forceDisplay: true);
        task.onLog += (EditorShell.LogType logType, string log) => {
            DockerLog(log);
        };
        task.onExit += (exitCode) => {
            DockerLog($"Docker compose down exit: {exitCode}", forceDisplay: true);
        };

        isRunning = false;
    }

    private static string BuildVolumesString(string[] volumes){

            if(volumes == null) return string.Empty;

            System.Text.StringBuilder builder = new System.Text.StringBuilder();
            
            foreach(var v in volumes){
                builder.Append(" -v ");
                builder.Append(v);
            }

            return builder.ToString();
    }

    public static void DockerRun(string service, 
                                 string command, 
                                 string[] additionalVolumes = null, 
                                 Action<int> callback = null) {

         // docker-compose -f ./docker/docker-compose.yml run --rm 
         // zosim_tools python ./zo-asset-tools/zo_convex_decomposition/zo_convex_decomposition.py
         var options = new EditorShell.Options(){
            workDirectory = "../../docker/",
            environmentVars = new Dictionary<string, string>(){
                //{"PATH", "usr/bin"}
            }
        };

        string volumes = BuildVolumesString(additionalVolumes);

        // Run command in a new container, and delete after execution with --rm
        string dockerCommand = $"docker-compose run --rm{volumes} {service} {command}";
        Debug.Log(dockerCommand);

        // Execute docker command
        var task = EditorShell.Execute(dockerCommand, options);
        DockerLog($"Executing command on {service}...", forceDisplay: true);
        task.onLog += (EditorShell.LogType logType, string log) => {
            DockerLog(log);
        };
        task.onExit += (exitCode) => {
            DockerLog($"Docker compose run exit: {service} ## {command}", forceDisplay: true);
            callback(exitCode);
        };
    }
    
    private string GetCommandPath(string command){
        System.Diagnostics.ProcessStartInfo getDockerPathProcessInfo = new System.Diagnostics.ProcessStartInfo();
        getDockerPathProcessInfo.FileName = "which";
        getDockerPathProcessInfo.Arguments = command;
        getDockerPathProcessInfo.UseShellExecute = false;
        getDockerPathProcessInfo.RedirectStandardError = true;
        getDockerPathProcessInfo.RedirectStandardOutput = true;

        System.Diagnostics.Process process = System.Diagnostics.Process.Start(getDockerPathProcessInfo);
        string output = process.StandardOutput.ReadToEnd();
        return output;
    }

    private static void DockerLog(string message, bool forceDisplay = false)
    {
        if(!showLogs && !forceDisplay) return;

        UnityEngine.Debug.Log($"<color={dockerLogColor}>{message}</color>");
        
    }
}
}