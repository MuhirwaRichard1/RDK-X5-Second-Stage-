; Inno Setup script for NavBot Console.
; Build via packaging\windows\build.ps1 (passes /DMyAppVersion=<ver>).
; Per-user install by default — no admin prompt needed.

#ifndef MyAppVersion
  #define MyAppVersion "0.0.0"
#endif
#define MyAppName "NavBot Console"
#define MyAppExeName "navbot-console.exe"

[Setup]
AppId={{C1EECF9B-1825-4C16-B205-77ADA20B85E7}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher=NavBot
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputDir=..\..\dist
OutputBaseFilename=navbot-console-{#MyAppVersion}-windows-setup
SetupIconFile=..\icon.ico
UninstallDisplayIcon={app}\{#MyAppExeName}
Compression=lzma2
SolidCompression=yes
WizardStyle=modern

[Tasks]
; checked by default — this is the desktop shortcut icon
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "..\..\dist\navbot-console\*"; DestDir: "{app}"; Flags: recursesubdirs ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent
