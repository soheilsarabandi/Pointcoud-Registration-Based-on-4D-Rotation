# This folder contains the Matlab code associated with the paper:

S. Sarabandi and F. Thomas, "Approximating Displacements in R^3 by Rotations 
in R^4 and its Application to Pointcloud Registation," submitted to IEEE TRO, 2021.






![Screenshot 2020-02-09 at 5 08 54 PM](https://www.researchgate.net/profile/Soheil-Sarabandi/publication/355936847/figure/fig1/AS:1086774742908929@1636118698190/left-Two-pointclouds-corresponding-to-two-partial-3D-scans-of-the-Stanford-Bunny-data_W640.jpg)

No proper norm exists to measure the distance between two object poses essentially because a general pose is defined by a rotation and a translation, and thus it involves magnitudes with different units. As a means to solve this dimensional-inhomogeneity problem, the concept of characteristic length has been put forward in the area of kinematics. The idea consists in scaling translations according to this characteristic length and then approximating the corresponding displacement defining the object pose in R^3 by a rotation in R^4, for which a norm exists. This paper sheds new light on this kind of approximations which permits simplifying optimization problem whose cost functions involve translations and rotations simultaneously. A good example of this kind of problems is the pointcloud registration problem in which the optimal rotation and translation between two sets of corresponding 3D point data, so that they are aligned/registered, have to be found. As a result, a simple closed-form formula for solving this problem is presented which is shown to be an attractive alternative to the previous approaches.
