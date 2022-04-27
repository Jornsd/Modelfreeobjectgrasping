# pcd2txt

## Brief

[![C++: solutions](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](https://es.wikipedia.org/wiki/C%2B%2B) [![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://wiki.ros.org/Industrial)

> C++ application to convert .pcd file (point cloud data) into txt format (xyzi).

## Input file structure

- .pcd

Including normals

Check your .pcd file with: `head <pcd file>`

## Output file structure

- model_cloud.txt

```
  x      y      z       nx     ny     nz     # position (not printable)
-0.1    0.1    0.1    
-0.2    0.2    0.2    
...     ...    ...    ...
```

Check with: `head model_cloud.txt`

Visualize 3D point cloud with:
[CloudCompare](https://cloudcompare.org/)

## Dependency

- [pcl](https://pointclouds.org/downloads/) (Point Cloud Library)

You can install pcl following these instructions:

https://cdmana.com/2021/12/20211203201022687d.html




## Compile

You can use the following commands to compile the package:

```
git clone https://github.com/Jornsd/Modelfreeobjectgrasping.git
cd /pcd2txt
mkdir build && cd build
cmake ../src/
make
```

## Run the package

Stand in the "build" folder and run:

```
cd ~/pcd2txt/build
./pcd2txt <pcd file> -o <output dir>
```


