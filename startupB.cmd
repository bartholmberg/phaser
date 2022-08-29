@echo off
chcp 65001
set junctionDir=o3d.jnc
set o3dRepoDir=..\Open3D\cpp
if not exist %o3dRepoDir% (
   echo you need to enlist %o3dRepoDir%  
   exit /b
)
IF not exist %junctionDir% ( 
	echo ........⏱️ 
	..\phaser\tool\junction -s -q -nobanner %junctionDir%  %o3dRepoDir%  
	rem and if you want to delete junction
	rem ..\phaser\tool\junction -d %junctionDir% 
	echo ⚽ All Done ✨  
	exit /b ) 