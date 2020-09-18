using System;
using System.Diagnostics;
using System.Threading.Tasks;

namespace ZO.Util {
    public static class ZOSystem {
        public static string RunProcessAndGetOutput(string workingDirectory, string executable, string arguments, out int exitCode) {
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


        public static async Task<int> RunProcessAsync(string fileName, string args, string workingDirectory = "./") {
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