## Image Stitching
![workflow](https://github.com/arghadeep25/Image-Stitching/actions/workflows/ci.yml/badge.svg)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/9296642e70004a5a891bcac45bfa7980)](https://app.codacy.com/gh/arghadeep25/Image-Stitching/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)
![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)
![Visitors](https://api.visitorbadge.io/api/visitors?path=https://github.com/arghadeep25/Image-Stitching)
![C++](https://forthebadge.com/images/badges/made-with-c-plus-plus.svg)
![Banner](https://socialify.git.ci/arghadeep25/Image-Stitching/image?description=1&font=Jost&forks=1&issues=1&language=1&owner=1&pulls=1&stargazers=1&theme=Auto)

### Description:

#### Sample Data (Berlin)
<p align="center">
  <img src="data/berlin/001.jpg" width="15%" /> 
  <img src="data/berlin/002.jpg" width="15%" />  
  <img src="data/berlin/003.jpg" width="15%" />
  <img src="data/berlin/004.jpg" width="15%">  
  <img src="data/berlin/005.jpg" width="15%"> 
  <img src="data/berlin/006.jpg" width="15%">  
</p> 

#### Results
<p align="center">
  <img src="results/berlin.jpg" width="%" />
</p>

### Build
```
git clone git@github.com:arghadeep25/Image-Stitching.git
cmake -S . -B build && cmake --build build -j$(nproc)
```

### Usage
```
./image_stitching -i <input_path> -o <output_path>
```
Example
```
./image_stitching -i ../data/berlin/ -o ../results/berlin.jpg
```
### Dependencies
 - OpenCV >= 4.5.1
 - C++ 17

### Methodology:
- #### Feature Extraction
    - Detect features from all the images.
    - Compute descriptors for each keypoint

- #### Feature Matching
    - Match features between all the images using a k-d tree to find the k-nearest neighbors in feature space.
    - Use a distance metric to determine the similarity between two descriptors.
    - Use a ratio test to determine the best matches.

- #### Image Matching
    - Use RANSAC to estimate the homography between two images.
    - Use the homography to warp the images.
    - Blend the images together.
    - Probabilistic Model

- #### Panorama Recognition
    - Connected Components

- #### Image Alignment
    - Homography Estimation
    - RANSAC
    - Image Warping
    - Image Blending

- #### Rendering
    - Automatic Straightening
    - Gain Compensation
    - Multi-band Blending

### Research Paper:

[Automatic Panoramic Image Stitching using Invariant Features](https://www.cs.ubc.ca/~lowe/papers/07brown.pdf)

### Additional Links:

- [OpenCV Image Stitching Description](https://docs.opencv.org/3.4/d1/d46/group__stitching.html)
- [PyImageSearch](https://pyimagesearch.com/2018/12/17/image-stitching-with-opencv-and-python/)