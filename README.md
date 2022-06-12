# Raytracer
A raytracer built from scratch in c++ using the OpenCV and Eigen libraries. Featuring shadow, refraction and reflection rays (using the hall shading model) as well as point lights, spot lights, polygonal meshes and spheres, limited 2D texture map support, adaptive supersampling, and the bounding volume heirarchy acceleration technique.

Images from each step of building the raytracer: https://www.cs.drexel.edu/~ae588/index.html

Code:
-raytracer.cpp - Main Raytracer code, runs independently
-raytracer_post_acceleration.cpp - Early BVH implementation of raytracer (see limtations section below), runs independently

# Setup for OpenCV and Eigen
Include Opencv build folder in the Include Directories and Opencv build/x64/vc15/lib folder in the library directories. The following link may help: https://learnopencv.com/code-opencv-in-visual-studio/#Step-4
Also include Eigen in the Additional Include Directories

# Limitations and bugs
-Texture maps are only enabled for spheres right now
-Only one 3D Procedural texture has been implemented (see the "tex.type ==" if conditions in getPixelColorFromRay)
-Cylinder entity is not fully implemented yet and is buggy
-Bounding Volume Heirarchy is not implemented in the main raytracer program (raytracer.cpp) but instead in raytracer_post_acceleration.cpp which only includes the phong shading model and adaptive supersampling. I had troulbe getting my BVH implementation to work well with shadow rays so I moved on without BVH fornow.

# Render examples:

![image](https://user-images.githubusercontent.com/15019257/173214602-657f7061-e246-4f00-ad83-0175637f8af2.png)
![image](https://user-images.githubusercontent.com/15019257/173214605-3abaa79f-0af3-410a-94ba-0d7e0bf9880c.png)
![image](https://user-images.githubusercontent.com/15019257/173214604-c17f9cde-8484-4d5f-a48e-92fe02dc0eb2.png)

