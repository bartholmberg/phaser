This is the Plan for integrating open3d point clouds into phaser. 

0   - Add Open3D_install to phaser .  This is the installation for o3d.  
   I've also update Open3D_install\lib with a runable list of libs to copy from o3d
1  - update   point-types.h with o3d pnt cld 
2  - Add new project 👉 phaser_o3d_driver that uses open3d classes, instead of PCL. (See PointCloud.cpp)
     Will use this to add in pieces of the original phaser_core_driver/driver.cc a 
	 little at a time
3  - BREAKS: phaser_core_driver project (for now) and it is removed from build
4  - Will use the 👉 PCL version of driver.cc to compare with new phaser_o3d_driver🏌️