Option Explicit

Dim fso, shell, userGuiDir, pythonw, mainScript, command
Set fso = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")

userGuiDir = fso.GetParentFolderName(WScript.ScriptFullName)
pythonw = fso.BuildPath(userGuiDir, ".venv\Scripts\pythonw.exe")
mainScript = fso.BuildPath(userGuiDir, "main.py")

If Not fso.FileExists(pythonw) Then
    MsgBox "Robot Companion is not installed. Run 1_INSTALL_AND_RUN_WINDOWS.bat first.", 16, "Robot Companion"
    WScript.Quit 1
End If

command = Chr(34) & pythonw & Chr(34) & " " & Chr(34) & mainScript & Chr(34)
shell.Run command, 0, False
