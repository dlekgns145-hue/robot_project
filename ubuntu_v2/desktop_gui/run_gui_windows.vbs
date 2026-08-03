Option Explicit

Dim fso, shell, guiDir, pythonw, mainScript, command
Set fso = CreateObject("Scripting.FileSystemObject")
Set shell = CreateObject("WScript.Shell")

guiDir = fso.GetParentFolderName(WScript.ScriptFullName)
pythonw = fso.BuildPath(guiDir, ".venv\Scripts\pythonw.exe")
mainScript = fso.BuildPath(guiDir, "main.py")

If Not fso.FileExists(pythonw) Then
    MsgBox "GUI is not installed. Run setup_gui_windows.ps1 first.", 16, "Robot Control v2"
    WScript.Quit 1
End If

command = Chr(34) & pythonw & Chr(34) & " " & Chr(34) & mainScript & Chr(34)
shell.Run command, 0, False
