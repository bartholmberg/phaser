
#  you need pcl repo and the following boost libs (for PCL)
#

vcpkg install boost-preprocessor:x64-windows
vcpkg install boost-assert:x64-windows
vcpkg install boost-mpl:x64-windows
vcpkg install boost-fusion:x64-windows
vcpkg install boost-numeric-conversion:x64-windows
 vcpkg install boost-algorithm:x64-windows
 vcpkg install boost-lexical-cast:x64-windows
 vcpkg install boost-interprocess:x64-windows
 vcpkg install pcl[vtk]:x64-windows --featurepackages --recurse