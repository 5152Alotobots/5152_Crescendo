# 5152_Crescendo
The Common Library for our FRC team contains code we reuse from year to year, like our drive subsystem.

This Repo is designed to eliminate extra hassel cleaning up our code and to make it easier to update wpilib

## Set up
#### 1. Clone this Repo (The Library)
   - Go to terminal type in `git clone https://github.com/5152Alotobots/5152_CommonLibrary.git`
#### 2. Update WPIlib (IF Nessisary)
   - Go to [https://github.com/wpilibsuite/allwpilib/releases](url) find the prerelease/release you want to update to.
   - Download ISO mount it and install WPIlib
   - Go to cloned common library repo and update it useing WPIlib update thing this should create a new folder called 5152_CommonLibrary-imported or something like that.
   - In THE **Non** Imported library delete everything except .vscode and vendorDeps (Make sure you are in VScode while doing this there are hidden files that you don't want to delete)
   - In the **Imported** library delete the .vscode and vendorDeps file
   - Copy Everything from Imported library into the Non Imported Library
   - Run a build/deploy to verify succses
   - **When You are Done Please add a Release of the Library so we can revert / reuse an older version of the library**
   - ***You Should make at least one realese a year***
#### 3. Install correct WPIlib (Not Nessisary is step one happend)
   - Please use the WPIlib installer (Sean) and install everything. to start the installer go to the most recent release of this repo and find the WPIlib ISO. Download it and extract it
#### 4. Fork this Repo (IF Not Already Done)
   - Press Fork
   - Change Owner to 5152Alotobots, and Change Name to 5152_(The Game Name Here), add discription
#### 5. Clone this years repo
   - If you had to fork it clone the repo you just made, If not clone the already existing repo either way the command will be the same
      - `git clone https://github.com/5152Alotobots/5152_(The Game Name Here).git`
##### 5.5 Hide Library folder 
   - To Prevent confusion we will hide the Library folder in the forked repo so you don't accidentaly edit it. Add this line to your settings.json file in the .vscode directory
```json
  {
  "java.configuration.updateBuildConfiguration": "automatic",
  "java.server.launchMode": "Standard",
  "files.exclude": {
    "**/.git": true,
    "**/.svn": true,
    "**/.hg": true,
    "**/CVS": true,
    "**/.DS_Store": true,
    "bin/": true,
    "**/.classpath": true,
    "**/.project": true,
    "**/.settings": true,
    "**/.factorypath": true,
    "**/*~": true,
    //THIS LINE BELOW
    "**/*/Library": true, 
  },
  "java.test.config": [
    {
      "name": "WPIlibUnitTests",
      "workingDirectory": "${workspaceFolder}/build/jni/release",
      "vmargs": [ "-Djava.library.path=${workspaceFolder}/build/jni/release" ],
      "env": {
        "LD_LIBRARY_PATH": "${workspaceFolder}/build/jni/release" ,
        "DYLD_LIBRARY_PATH": "${workspaceFolder}/build/jni/release"
      }
    },
  ],
  "java.test.defaultConfig": "WPIlibUnitTests"
}
```


    
