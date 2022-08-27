@echo off
chcp 65001
set junctionDir=pntphr.jnc
set phaserRepoDir=c:\repo\phaser
if not exist %phaserRepoDir% (
   echo you need to enlist %phaserRepoDir%  
   exit /b
)
IF not exist %junctionDir% ( 
	echo ........⏱️ 
	..\apricus\tool\junction -s -q -nobanner %junctionDir%  %phaserRepoDir%  
	rem and if you want to delete junction
	rem ..\apricus\tool\junction -d %junctionDir% 
	echo ⚽ All Done ✨  
	exit /b ) 

