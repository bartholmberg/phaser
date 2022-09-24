This is the Plan 🤟🏼 for integrating open3d point clouds into phaser. 

0   - Add Open3D_install to phaser .  This is the installation for o3d.  
      I've also update Open3D_install\lib with a runable list of libs to copy from o3d
1  - update   point-types.h with o3d pnt cld 
2  - Add new project 👉 phaser_o3d_driver that uses open3d classes, instead of PCL. (See PointCloud.cpp)
     Will use this to add in pieces of the original phaser_core_driver/driver.cc a 
	 little at a time
3  - BREAKS: phaser_core_driver project (for now) and it is removed from build
4  - Will use the 👉 PCL version of driver.cc to compare with new phaser_o3d_driver🏌️

(also search google for 'Can a exe link to DLL and static library at the same time?')

NOTE0:  we may not change to link with static libs.  Works fine linking with VTK DLLs in
in VCPKG.  So I *think* that the only time we need to build with the dynamic run-time library 
option is if we are actually creating making a DLL.

NOTES: on changing an app to static build:


Look for 👉🏽 Globals in .vcxproj of interest 

add the following line to bottom
<VcpkgTriplet Condition="'$(Platform)'=='x64'">x64-windows-static</VcpkgTriplet>

<PropertyGroup Label="Globals">
    <ProjectGuid>{0D8F2DCD-1F2E-3087-8AB2-711413C52C2B}</ProjectGuid>
    <WindowsTargetPlatformVersion>10.0.19041.0</WindowsTargetPlatformVersion>
    <Keyword>Win32Proj</Keyword>
    <Platform>x64</Platform>
    <ProjectName>phaser_core_driver</ProjectName>
    <VCProjectUpgraderObjectName>NoUpgrade</VCProjectUpgraderObjectName>
///  see 👀 it 👇🏼 gets added right here 👇🏼
	<VcpkgTriplet Condition="'$(Platform)'=='x64'">x64-windows-static</VcpkgTriplet>
  </PropertyGroup>
  
  (https://levelup.gitconnected.com/how-to-statically-link-c-libraries-with-vcpkg-visual-studio-2019-435c2d4ace03)




