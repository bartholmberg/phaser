@echo off
chcp 65001
set junctionDir=apricus.jnc
set apricusRepoDir=..\apricus
if not exist %apricusRepoDir% (
   echo you need to enlist %apricusRepoDir%  
   exit /b
)
IF not exist %junctionDir% ( 
	echo ........⏱️ 
	..\phaser\tool\junction -s -q -nobanner %junctionDir%  %apricusRepoDir%  
	rem and if you want to delete junction
	rem ...\phaser\tool\junction -d %junctionDir% 
	echo ⚽ All Done ✨  
	exit /b ) 