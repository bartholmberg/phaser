@echo off
chcp 65001
set junctionDir=apricus.jnc
set phaserRepoDir=c:\repo\apricus
if not exist %phaserRepoDir% (
   echo you need to enlist %phaserRepoDir%  
   exit /b
)
IF not exist %junctionDir% ( 
	echo ........⏱️ 
	..\phaser\tool\junction -s -q -nobanner %junctionDir%  %phaserRepoDir%  
	rem and if you want to delete junction
	rem ..\apricus\tool\junction -d %junctionDir% 
	echo ⚽ All Done ✨  
	exit /b ) 