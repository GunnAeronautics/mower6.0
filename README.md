# mower6.0
How to install build onto rasberry pi pico:

Get Vscode

Get Git

Install the extension platformio

Download the repository

Enable LongPaths:

For windows:

Open Registry Editor,

go to:

HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\FileSystem.

Select the entry named: LongPathsEnabled. Set the value to 1

Run Git CMD as administrator

Run this command:

git config --system core.longpaths true

Restart your computer

Open repo in platformio and it should start to build.

Download zadig driver installation tool if you want to upload more easily
