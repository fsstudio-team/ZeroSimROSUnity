using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

// Create a new type of Settings Asset.
class ZOSettings : ScriptableObject
{
    public const string _settingsPath = "Assets/ZOSim/Editor/";
    public const string _settingsFilename = "ZOSettings.asset";

    [SerializeField]
    private string _dockerComposeWorkingDirectory;

    // PROPERTIES
    public string ComposeWorkingDirectory { get{ return _dockerComposeWorkingDirectory; } }

    internal static ZOSettings GetOrCreateSettings()
    {
        var settings = AssetDatabase.LoadAssetAtPath<ZOSettings>(_settingsPath + _settingsFilename);
        if (settings == null)
        {
            settings = ScriptableObject.CreateInstance<ZOSettings>();

            // DEFAULT VALUES FOR SETTINGS
            // Search for a docker-compose.yml file
            string[] dockerComposeAssets = AssetDatabase.FindAssets("docker-compose");
            if(dockerComposeAssets.Length == 0){
                // if not found, assume we're in development and set the default dir for development
                settings._dockerComposeWorkingDirectory = "../../docker/dev";
            }
            else {
                foreach(var dockerComposeAsset in dockerComposeAssets){ // find the first .yml and use that one (need to iterate through results because folders also appear in this array)
                    string filePath = AssetDatabase.GUIDToAssetPath(dockerComposeAssets[0]);
                    if(filePath.EndsWith(".yml")){
                        string directory = System.IO.Path.GetDirectoryName(filePath);
                        settings._dockerComposeWorkingDirectory = directory;
                        break;
                    }
                }
            }
            
            // create directory if not exists
            System.IO.Directory.CreateDirectory(_settingsPath);
            
            Debug.Log("Created new ZO settings");
            AssetDatabase.CreateAsset(settings, _settingsPath + _settingsFilename);
            AssetDatabase.SaveAssets();
        }
        return settings;
    }

    internal static SerializedObject GetSerializedSettings()
    {
        return new SerializedObject(GetOrCreateSettings());
    }
}

// Register a SettingsProvider using IMGUI for the drawing framework:
static class ZOSettingsIMGUIRegister
{
    [SettingsProvider]
    public static SettingsProvider CreateZOSettingsProvider()
    {
        // First parameter is the path in the Settings window.
        // Second parameter is the scope of this setting: it only appears in the Project Settings window.
        var provider = new SettingsProvider("Project/ZeroSim Settings", SettingsScope.Project)
        {
            // By default the last token of the path is used as display name if no label is provided.
            label = "ZeroSim",
            // Create the SettingsProvider and initialize its drawing (IMGUI) function in place:
            guiHandler = (searchContext) =>
            {
                var settings = ZOSettings.GetSerializedSettings();

                if(GUILayout.Button("Select docker-compose.yml working directory")){
                    string pathDockerCompose = EditorUtility.OpenFolderPanel("Select docker-compose.yml directory", "", "");
                    if(!string.IsNullOrEmpty(pathDockerCompose)){
                        
                        settings.FindProperty("_dockerComposeWorkingDirectory").stringValue = pathDockerCompose;
                        
                        Debug.Log(settings.FindProperty("_dockerComposeWorkingDirectory").stringValue);
                        if(settings.ApplyModifiedProperties()) Debug.Log("Updated setting docker dir");
                    }
                }
                EditorGUILayout.LabelField("Docker-compose workdir:", settings.FindProperty("_dockerComposeWorkingDirectory").stringValue);
                
            },

            // Populate the search keywords to enable smart search filtering and label highlighting:
            keywords = new HashSet<string>(new[] { "docker", "compose" })
        };

        return provider;
    }
}