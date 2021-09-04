# 3D reconstruction

In this project, I have implemented a simple 3d reconstruction algorithm that uses a calibrated camera and the known 3d points to reconstruct the scnene (Figure 1).

![Result](simple_reconstruction_yuto1.png?raw=true "Result")
*Figure 1. Reconstructed 3d face.*

I put a checkerboard in the background and took two pictures of my son. Then, I manually selected over 100 matching points in two pictures. Then, the system computed the 3d point cloud by linear triangulation. For details about this method, please refer to Chapter 12 of the book <i>Multiple View Geometry in Computer Vision</i> by Hartley and Zisserman. Once 3d point cloud were generated, Delaunay triangulation was used to generate the mesh triangulation (Figure 2).

![Result](simple_reconstruction_yuto1_wireframe.png?raw=true "Result")
*Figure 2. Generated mesh triangulation.*

Finally, the original image was wapred for each triangle to generate textures. To define the affine transformation matrix, each 3d triangle was first transformed to 2d triangle <i>T'</i>. To do this, the first edge of the 3d triangle was used as X axis and Y and Z axis are defiend based on that. The matrix [X, Y, Z].inv() is automatically the transformation matrix that transform the 3d triangle to 2d triangle <i>T'</i>. Then, the affine transformation matrix which transform the original corresponding triangle <i>T</i> to this triangle <i>T'</i> was computed. After the affine transformation matrix <i>M</i> was computed, the original image was warped by <i>M</i> and transferred to GPU by <i>texImage2D()</i> method. The coordinates of each vertex of <i>T'</i> are already the texture coordinates. Once the 3d object is reconstructed, the object can be easily rotated and translated by moving the camera (Figure 3).

![Result](simple_reconstruction_yuto2.png?raw=true "Result")
*Figure 3. Rotated 3d face.*

Limitations:</strong> Because of some measurement errors, the triangulated 3d points have some errors in their coordinates. Also, over 100 points are still not enough for complex surface like human face. You may notice some artifacts in Figure 4.

![Result](simple_reconstruction_yuto3.png?raw=true "Result")
*Figure 4. Some artifacts on the 3d face.*
