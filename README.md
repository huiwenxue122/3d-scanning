# 3d-scanning-demos

## Laser Line Detection

The core algorithm (`detectLaser`) enhances the red channel and detects the vertical laser line as follows:

1. Apply Gaussian blur to smooth the input image.  
2. Compute the red response for each pixel:  
   \[
   v = r - \tfrac{g+b}{2}, \quad v \geq 0
   \]
3. For each row, find the maximum response (`maxRow[h]`).  
4. Compute the global average maximum (`avgMax`) and define a threshold `T = 0.8 * avgMax`.  
5. Keep only the peak pixel per row if its value ≥ T, set others to 0.

The figure below illustrates the process:

- **Orange line** = row maxima (`maxRow[h]`)  
- **Orange dashed line** = global average (`avgMax`)  
- **Red dashed line** = threshold (`0.8 * avgMax`)  
- **Green points** = rows where a valid laser peak is detected  

![detectLaser explanation](your_image_path.png)
