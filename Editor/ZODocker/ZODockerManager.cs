using System;
using System.Text.RegularExpressions;
using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using MS.Shell.Editor;
using UnityEditor;
using UnityEngine;

namespace ZO.Editor {

    public class ZODockerManager {
        public static string dockerLogColor = "#207020";
        public static bool showLogs = true;
        public static bool isInstalled = false;
        public static bool isRunning = false;
        public static string composeWorkingDirectory = "../../docker/dev"; // default Compose file for development

        private static readonly string installScriptAssetName = "docker_install"; // need this for AssetDatabase to find it
        private static readonly string installScriptName = "docker_install.sh";

        /// <summary>
        /// Install Docker.  Note:  Probably only works for Linux.
        /// </summary>
        public static void DockerInstall() {

            // Find Docker install script
            string[] installScriptAssets = AssetDatabase.FindAssets(installScriptAssetName);
            if (installScriptAssets.Length == 0) {
                EditorUtility.DisplayDialog("Error", "Couldn't find docker_install.sh among your assets. Have you imported ZeroSim samples already?", "Ok");
                return;
            }

            string scriptPath = AssetDatabase.GUIDToAssetPath(installScriptAssets[0]);
            string directory = System.IO.Path.GetDirectoryName(scriptPath);
            UnityEngine.Debug.Log(directory);

            // Navigate to parent directories where the docker and dockercompose files are located
            var options = new EditorShell.Options() {
                workDirectory = directory
            };
            string command = "./" + installScriptName;
            // Execute docker command
            var task = EditorShell.Execute(command, options);
            task.onLog += (EditorShell.LogType logType, string log) => {
                DockerLog(log);
            };
            task.onExit += (exitCode) => {
                if (exitCode == 0) {
                    isRunning = true;
                }

                DockerLog($"Install script exit code: {exitCode}", forceDisplay: true);
            };
        }

        /// <summary>
        /// Determine if docker is installed.
        /// </summary>
        /// <returns></returns>
        public static Task<bool> IsZODockerInstalled() {
            var options = new EditorShell.Options() {
                workDirectory = Application.dataPath
            };

            // Create a task and return it so clients can use async/await
            // Use TaskCompletionSource so that we can manually fulfill the task when 
            // the shell script executes the onExit callback
            TaskCompletionSource<bool> taskCompletionSource = new TaskCompletionSource<bool>();

            string command = "docker --version && docker-compose --version";
            // Execute docker command
            var shellTask = EditorShell.Execute(command, options);
            DockerLog($"Checking if docker installed...");
            shellTask.onLog += (EditorShell.LogType logType, string log) => {
                //DockerLog(log);
            };
            shellTask.onExit += (exitCode) => {
                //DockerLog("Check if docker installed exit code: " + exitCode);
                isInstalled = exitCode == 0;
                taskCompletionSource.SetResult(isInstalled);
            };

            return taskCompletionSource.Task;
        }

        public static Task<bool> IsZODockerRunning() {
            ZOSettings settings = ZOSettings.GetOrCreateSettings();

            var options = new EditorShell.Options() {
                workDirectory = settings.ComposeWorkingDirectory,
                environmentVars = new Dictionary<string, string>() { }
            };

            // Create a task and return it so clients can use async/await
            // Use TaskCompletionSource so that we can manually fulfill the task when 
            // the shell script executes the onExit callback
            TaskCompletionSource<bool> taskCompletionSource = new TaskCompletionSource<bool>();

            string command = "if [ $(docker inspect -f '{{.State.Running}}' zosim) = \"true\" ]; then exit 0; else exit 1; fi";
            // Execute docker command
            var shellTask = EditorShell.Execute(command, options);
            DockerLog($"Checking if docker running...");
            shellTask.onLog += (EditorShell.LogType logType, string log) => {
                DockerLog(log);
            };
            shellTask.onExit += (exitCode) => {
                UnityEngine.Debug.Log("Check if docker running exit code: " + exitCode);
                isRunning = exitCode == 0;
                taskCompletionSource.SetResult(isRunning);
            };

            return taskCompletionSource.Task;
        }

        public static void DockerComposeUp() {
            ZOSettings settings = ZOSettings.GetOrCreateSettings();

            // Navigate to parent directories where the docker and dockercompose files are located
            var options = new EditorShell.Options() {
                workDirectory = settings.ComposeWorkingDirectory,
                environmentVars = new Dictionary<string, string>() { }
            };

            // need to set environment variable so that we execute container as current user in host machine
            string command = "CURRENT_UID=$(id -u):$(id -g) docker-compose up";
            // Execute docker command
            var task = EditorShell.Execute(command, options);
            DockerLog($"Starting docker container, please wait...");
            task.onLog += (EditorShell.LogType logType, string log) => {
                DockerLog(log);
            };
            task.onExit += (exitCode) => {
                if (exitCode == 0) {
                    // we won't get this until the server is stopped
                }

                DockerLog($"Docker compose up exit: {exitCode}", forceDisplay: true);
            };

            // TODO: poll to check if the server was started successfully
            // here we assume it will
            isRunning = true;
        }

        public static void DockerComposeDown() {
            ZOSettings settings = ZOSettings.GetOrCreateSettings();

            // Navigate to parent directories where the docker and dockercompose files are located
            var options = new EditorShell.Options() {
                workDirectory = settings.ComposeWorkingDirectory,
                environmentVars = new Dictionary<string, string>() { }
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

        private static string BuildVolumesString(string[] volumes) {

            if (volumes == null) return string.Empty;

            System.Text.StringBuilder builder = new System.Text.StringBuilder();

            foreach (var v in volumes) {
                builder.Append(" -v ");
                builder.Append(v);
            }

            return builder.ToString();
        }


        /// <summary>
        /// Docker compose run command.  
        /// <see>https://docs.docker.com/compose/reference/run/</see>
        /// </summary>
        /// <param name="service">The Docker Service.  For example "zosim".</param>
        /// <param name="command">The docker command to execute.</param>
        /// <param name="additionalVolumes">Additional volumes to mount.</param>
        /// <param name="servicePorts">Run command with the service's ports enabled and mapped to the host.</param>
        /// <param name="setUser">Run as specified username or uid.</param>
        /// <param name="callback">The on exit callback.</param>
        public static void DockerRun(string service,
                                     string command,
                                     string[] additionalVolumes = null,
                                     bool doServicePorts = true,
                                     bool setUser = true,
                                     Action<int> callback = null) {

            

            ZOSettings settings = ZOSettings.GetOrCreateSettings();

            // docker-compose -f ./docker/docker-compose.yml run --rm 
            // zosim_tools python ./zo-asset-tools/zo_convex_decomposition/zo_convex_decomposition.py
            var options = new EditorShell.Options() {
                workDirectory = settings.ComposeWorkingDirectory,
                environmentVars = new Dictionary<string, string>() { }
            };

            // set up the volumes to mount
            string volumes = BuildVolumesString(additionalVolumes);

            // set up server ports
            string servicePorts = doServicePorts ? "--service-ports" : "";

            // set up user
            // --user=$(id -u $USER):$(id -g $USER)
            string userName = System.Environment.GetEnvironmentVariable("USER");
            string userParam = "";
            if (setUser == true) {
                int exitCode = -99;
                string userId = RunProcessAndGetOutput("./", "id", $"-u {userName}", out exitCode);
                userId = Regex.Replace(userId, @"\s+", string.Empty);
                string userGroup = RunProcessAndGetOutput("./", "id", $"-g {userName}", out exitCode);
                userGroup = Regex.Replace(userGroup, @"\s+", string.Empty);
                userParam = $"--user={userId}:{userGroup}";
            }

            // Run command in a new container, and delete after execution with --rm
            string dockerCommand = $"docker-compose run --rm {volumes} {servicePorts} {userParam} {service} {command}";
            UnityEngine.Debug.Log(dockerCommand);

            string arguments = $"run --rm {volumes} {servicePorts} {userParam} {service} {command}";
            Task<int> t = RunProcessAsync("docker-compose", arguments, settings.ComposeWorkingDirectory);
        }

        /// <summary>
        /// Get the path to a command.
        /// </summary>
        /// <param name="command"></param>
        /// <returns></returns>    
        private string GetCommandPath(string command) {
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

        private static void DockerLog(string message, bool forceDisplay = false) {
            if (!showLogs && !forceDisplay) return;

            UnityEngine.Debug.Log($"<color={dockerLogColor}>{message}</color>");

        }

        static string RunProcessAndGetOutput(string workingDirectory, string executable, string arguments, out int exitCode) {
            using (var p = new Process()) {
                p.StartInfo.WorkingDirectory = workingDirectory;
                p.StartInfo.FileName = executable;
                p.StartInfo.Arguments = arguments;
                p.StartInfo.UseShellExecute = false;
                p.StartInfo.CreateNoWindow = true;
                p.StartInfo.RedirectStandardOutput = true;
                p.StartInfo.RedirectStandardError = true;
                var output = string.Empty;
                p.OutputDataReceived += (sender, e) => {
                    output += $"{e.Data}{Environment.NewLine}";
                    UnityEngine.Debug.Log(e.Data);
                };
                p.ErrorDataReceived += (sender, e) => { output += $"{e.Data}{Environment.NewLine}"; };
                p.Start();
                p.BeginOutputReadLine();
                p.BeginErrorReadLine();
                p.WaitForExit();
                exitCode = p.ExitCode;
                return output;
            }
        }


        public static async Task<int> RunProcessAsync(string fileName, string args, string workingDirectory) {
            using (var process = new Process {
                StartInfo = {
                    FileName = fileName,
                    Arguments = args,
                    WorkingDirectory = workingDirectory,
                    UseShellExecute = false,
                    CreateNoWindow = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true
                },
                EnableRaisingEvents = true
            }) {
                return await RunProcessAsync(process).ConfigureAwait(false);
            }

        }

        public static Task<int> RunProcessAsync(Process process) {
            var tcs = new TaskCompletionSource<int>();

            process.Exited += (s, ea) => tcs.SetResult(process.ExitCode);
            process.OutputDataReceived += (s, ea) => UnityEngine.Debug.Log(ea.Data);
            process.ErrorDataReceived += (s, ea) => UnityEngine.Debug.LogError("ERROR: " + ea.Data);

            bool started = process.Start();
            if (!started) {
                //you may allow for the process to be re-used (started = false) 
                //but I'm not sure about the guarantees of the Exited event in such a case
                throw new InvalidOperationException("Could not start process: " + process);
            }

            process.BeginOutputReadLine();
            process.BeginErrorReadLine();

            return tcs.Task;
        }
    }
}