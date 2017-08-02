# README #

The idea is to compute keypoints and their correspondences between frames, and then to the set of homographies that best explains all the keypoints. In principle, if there is motion in the image stream, each homography should correspond to a distinct surface.

In practice, this does not seem to work; the system seems to get overwhelmed with noise under any reasonable parameterization.
