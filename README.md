# 3d-scanning-demos

## Laser Line Detection

The `detectLaser` function detects a vertical red laser line in an image through the following steps:

1. **Smoothing**: Apply Gaussian filtering to reduce noise.  
2. **Red channel response**: Compute `v = r - (g+b)/2`, which highlights red pixels.  
3. **Row maxima**: For each row, find the maximum response value.  
4. **Adaptive thresholding**: Compute a global average maximum (`avgMax`) and use `T = 0.8 * avgMax` to filter out weak responses.  
5. **Line extraction**: For each row, keep only the peak pixel if above threshold; set all others to black.

### Example

Input image with intensity profile (before and after filtering):

![Input and filtering result](2.png)

Final detected laser line (binary output):

![Laser line detection result](3.png)


The core algorithm (`detectLaser`) enhances the red channel and detects the vertical laser line as follows:

1. Apply Gaussian blur to smooth the input image.  
2. Compute the red response for each pixel:  
  `v = r - (g + b) / 2,  v >= 0`
3. For each row, find the maximum response (`maxRow[h]`).  
4. Compute the global average maximum (`avgMax`) and define a threshold `T = 0.8 * avgMax`.  
5. Keep only the peak pixel per row if its value ≥ T, set others to 0.

The figure below illustrates the process:

- **Orange line** = row maxima (`maxRow[h]`)  
- **Orange dashed line** = global average (`avgMax`)  
- **Red dashed line** = threshold (`0.8 * avgMax`)  
- **Green points** = rows where a valid laser peak is detected  

![detectLaser explanation](1.png)

# Assignment 02 | Camera Calibration and Optical Triangulation  

This project was developed as part of **ENGN2502: 3D Photography (Fall 2024, Brown University)**.  
The goal is to implement **camera calibration** and **laser line triangulation** to reconstruct 3D point clouds from structured light scanning.  

---

## 📷 Camera Calibration
- Used the MATLAB Camera Calibration Toolbox to estimate intrinsic and extrinsic parameters.  
- Validated calibration accuracy by implementing a reprojection function in C++ with Eigen.  
- Achieved reprojection error (RMSE) < **0.5 pixels**.  

---

## 🔺 Optical Triangulation
- Implemented **ray–plane intersection** in C++ to compute 3D points from detected laser line pixels.  
- Converted points from the camera coordinate system to the world coordinate system, accounting for turntable rotation.  
- Generated 3D point clouds of scanned objects.  

**Pipeline:**  
1. Detect laser line (`hw1::detectLaser`).  
2. Back-project pixels to rays using camera intrinsics.  
3. Intersect rays with the laser plane.  
4. Transform results into world coordinates.  
5. Undo turntable rotation to align all scans.  

---

## 🛠️ Technologies
- **C++** (Eigen, Qt)  
- **MATLAB** (Camera Calibration Toolbox)  
- **Meshlab** (point cloud visualization)  

---

## 📊 Results
Below is an example of the reconstructed **3D point cloud** of a scanned vase:  

![Point Cloud Result](6.png)  

---

