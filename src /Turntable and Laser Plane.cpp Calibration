#include <iostream>

#include "homework3.hpp"
// #include "homework2.hpp"
#include "homework1.hpp"

#include <Eigen/SVD>
#include <math.h>

using namespace Eigen;

// Turntable Calibration

void hw3::estimateExtrinsics
(Matrix3d const& K, /* intrinsic */
 Vector5d const& kc, /* intrinsic */
 QVector<Vector2d> const& worldCoord, /* input */
 QVector<Vector2d> const& imageCoord, /* input, distorted */
 Matrix3d& R, /* output */
 Vector3d& T  /* output */) {

  (void)kc; // ignore lens distortion for this assignmet

  // (R,T) to be estimated
  //
  // [r0 r1 r2] = R
  // R*p+T = r0 * px + r1 * py + T
  //
  // for each p=(px,py,0)^t in worldCoord 

  int nPoints = worldCoord.size();

  // create a matrix A of dimension 2*nPoints rows X 9 columns
  MatrixXd A(2 * nPoints, 9);

  // for each of the nPoints p=(x,y,0)^t in worldCoord  {
  for (int i = 0; i < nPoints; ++i) {
  //   - get the corresponding image point in pixel coords from imageCoordC
      Vector2d u_dist_pix = imageCoord[i];
  //   - compute the normalized distorted image point coordinates by
  //     solving : K*u_dist_norm = u_dist_pix
  //
  //     [ k_00 k_01 k_02 ] [ u_dist_norm(0) ]          [ u_dist_pix(0) ]
  //     [   0  k_11 k_12 ] [ u_dist_norm(1) ] = lambda [ u_dist_pix(1) ]
  //     [   0    0  k_22 ] [       1        ]          [        1      ]
      Vector3d u_dist_pix_homogeneous(u_dist_pix(0), u_dist_pix(1), 1.0);
      Vector3d u_dist_norm = K.inverse() * u_dist_pix_homogeneous;

  //   - remove distortion (ignored here)
  //     to obtain normailized camera coordinates (ux,uy)
      double ux = u_dist_norm(0);
      double uy = u_dist_norm(1);

      double px = worldCoord[i][0];
      double py = worldCoord[i][1];
  //   - add the following two rows to the matrix A
  //     [  0  -px  uy*px  0  -py  uy*py  0 -1  uy ]
  //     [  px  0  -ux*px  py  0  -ux*py  1  0 -ux ]
      A(2 * i, 0) = 0;
      A(2 * i, 1) = -px;
      A(2 * i, 2) = uy * px;
      A(2 * i, 3) = 0;
      A(2 * i, 4) = -py;
      A(2 * i, 5) = uy * py;
      A(2 * i, 6) = 0;
      A(2 * i, 7) = -1;
      A(2 * i, 8) = uy;

      A(2 * i + 1, 0) = px;
      A(2 * i + 1, 1) = 0;
      A(2 * i + 1, 2) = -ux * px;
      A(2 * i + 1, 3) = py;
      A(2 * i + 1, 4) = 0;
      A(2 * i + 1, 5) = -ux * py;
      A(2 * i + 1, 6) = 1;
      A(2 * i + 1, 7) = 0;
      A(2 * i + 1, 8) = -ux;

  }
  //
  // - compute
  //   X = argmin \| AX \|^2 under the cnstarint \|X\|^2=1
  //
  // - solution is right singular vector corresponding to the minimum
  //   singular value of A
  // - can use Eigen::JacobiSVD to perform this computation
  JacobiSVD<MatrixXd> svd(A, ComputeFullV);
  VectorXd X = svd.matrixV().col(8);

  // - except for a multiplicative normalization factor, the vector X
  //   should be equal to the concatenation of the vectors r0, r1, and
  //   t
  Vector3d r0(X(0), X(1), X(2));
  Vector3d r1(X(3), X(4), X(5));
  Vector3d t(X(6), X(7), X(8));

  // - the normalization factor can be determined by the fact that r0
  //   and r1, being the first two columns of a rotation matrix,
  //   should be unit length vectors and orthogonal to each other (how
  //   do we deal with the fact that here we have three constraints,
  //   but we can impose only one to determine the normalization factor?)
  double norm_r0 = r0.norm();
  double norm_r1 = r1.norm();
  double scale = std::sqrt(norm_r0 * norm_r1);

  // - find a way to measure how far are the resulting matrix R is
  // - from an orthogonal matrix, and print the value
  //
  // - once the normalization factor is determined, X should be
  //   normalized before t is extracted
  r0 /= scale;
  r1 /= scale;
  t /= scale;

  // - also the third column r2 of the matrix R can be computed as the
  //   cross product of r0 and r1
  Vector3d r2 = r0.cross(r1);

  R.col(0) = r0;
  R.col(1) = r1;
  R.col(2) = r2;

  Matrix3d orthogonality = R.transpose() * R - Matrix3d::Identity();
  double orthogonality_error = orthogonality.norm();

  std::cout << "Orthogonality error of R: " << orthogonality_error << std::endl;

  Eigen::JacobiSVD<Matrix3d> svd_R(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  R = svd_R.matrixU() * svd_R.matrixV().transpose();

  r0 = R.col(0);
  r1 = R.col(1);
  r2 = R.col(2);

  T = t;
}

//////////////////////////////////////////////////////////////////////
void hw3::projectPoints
(Matrix3d          const& K,
 Vector5d          const& kc, 
 Matrix3d          const& R_world,
 Vector3d          const& T_world,
 QVector<Vector3d> const& worldPoints, 
 QVector<Vector2d>      & projectedPoints) {
  
  // - derived from hw2::reprojectPoints() by removing the image
  //   points detected by the calibration software used to measure the
  //   rms error
  projectedPoints.clear();

  int nPoints = worldPoints.size();

  for (int i = 0; i < nPoints; ++i) {

      const Vector3d& p = worldPoints[i];

      Vector3d pc = R_world * p + T_world;

      Vector2d u = pc.hnormalized();

  // - remember to apply the lens distortion model and to convert to
  //    pixel coordinates
      double x = u(0);
      double y = u(1);
      double r2 = x * x + y * y;

      double radial_dist = 1 + kc[0] * r2 + kc[1] * r2 * r2 + kc[4] * r2 * r2 * r2;
      double x_distorted = x * radial_dist + 2 * kc[2] * x * y + kc[3] * (r2 + 2 * x * x);
      double y_distorted = y * radial_dist + kc[2] * (r2 + 2 * y * y) + 2 * kc[3] * x * y;

      Vector2d dist_u(x_distorted, y_distorted);

      Vector3d dist_u_homogeneous(dist_u(0), dist_u(1), 1.0);
      Vector3d projectedPointHomogeneous = K * dist_u_homogeneous;
      Vector2d projectedPoint = projectedPointHomogeneous.hnormalized();

      projectedPoints.push_back(projectedPoint);
  }

}

//////////////////////////////////////////////////////////////////////
void hw3::projectPoints
(Matrix3d          const& K, /* input */
 Vector5d          const& kc, /* input */
 QVector<Vector3d> const& cameraPoints, /* input */  
 QVector<Vector2d>      & imagePoints /* output */ ) {

  // - same as the previous version when the camera coordinate system
  //   is the world coordinate system; i.e., when R_world ==
  //   Matrix3d::Identity() and T_world == Vector3d::Zero()

    imagePoints.clear();

    int nPoints = cameraPoints.size();

    for (int i = 0; i < nPoints; ++i) {

        const Vector3d& pc = cameraPoints[i];

        Vector2d u = pc.hnormalized();

        double x = u(0);
        double y = u(1);
        double r2 = x * x + y * y;

        double radial_dist = 1 + kc[0] * r2 + kc[1] * r2 * r2 + kc[4] * r2 * r2 * r2;
        double x_distorted = x * radial_dist + 2 * kc[2] * x * y + kc[3] * (r2 + 2 * x * x);
        double y_distorted = y * radial_dist + kc[2] * (r2 + 2 * y * y) + 2 * kc[3] * x * y;

        Vector2d dist_u(x_distorted, y_distorted);

        Vector3d dist_u_homogeneous(dist_u(0), dist_u(1), 1.0);
        Vector3d imagePointHomogeneous = K * dist_u_homogeneous;
        Vector2d imagePoint = imagePointHomogeneous.hnormalized();

        imagePoints.push_back(imagePoint);
    }
}

// Laser Plane Calibration

//////////////////////////////////////////////////////////////////////
void hw3::detectLaser
(QImage const& inputImage, /* input */
 QVector<Vector2d>& laserLineCoord, /* output */
 int x0, int x1, int y0, int y1) {

  // - closely related to
  //   hw1::detectLaser(QImage const& inputImage);
  //
  // - for each row in the range [y0:y1] of the input image
  //   if a laser point is detected on that row (assumed to be at most one)
  //     an the location of the detected point is in the the range [x0:x1]
  //     - add the (x,y) coordinates to the output vector
  //
  // - how can you estimate the x coordinate at subpixel resolution ?

    laserLineCoord.clear();

    // 检查图像格式
    if (inputImage.format() != QImage::Format_RGB32) {
        return; // 不支持的图像格式
    }

    // 确保坐标范围合法
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(inputImage.width() - 1, x1);
    y1 = std::min(inputImage.height() - 1, y1);

    // 应用平滑滤波器以减少噪声
    QVector<int> filter(QVector<int>() << 1 << 4 << 6 << 4 << 1);
    QImage smoothedImage = hw1::filterImage(inputImage, filter);

    int height = smoothedImage.height();

    // 创建一个向量来存储每行的最大强度值
    QVector<double> maxValuesPerRow(height, 0.0);

    // 第一步：遍历指定的行范围 [y0, y1]，收集每行的最大强度值
    for (int h = y0; h <= y1; ++h) {
        for (int w = x0; w <= x1; ++w) {
            QRgb pixel = smoothedImage.pixel(w, h);
            int r = qRed(pixel);
            int g = qGreen(pixel);
            int b = qBlue(pixel);

            // 计算红色通道与其他通道的差异
            double value = r - (g + b) / 2.0;
            value = std::clamp(value, 0.0, 255.0);

            if (value > maxValuesPerRow[h]) {
                maxValuesPerRow[h] = value;
            }
        }
    }

    // 计算平均最大值，用于动态阈值
    double avgMax = std::accumulate(maxValuesPerRow.begin() + y0, maxValuesPerRow.begin() + y1 + 1, 0.0) / (y1 - y0 + 1);

    // 第二步：根据动态阈值检测激光点
    for (int h = y0; h <= y1; ++h) {
        if (maxValuesPerRow[h] < 0.5 * avgMax) {
            continue; // 如果这一行的最大值低于阈值，跳过该行
        }

        double maxVal = 0.0;
        int maxPos = -1;

        // 遍历指定的列范围 [x0, x1]
        for (int w = x0; w <= x1; ++w) {
            QRgb pixel = smoothedImage.pixel(w, h);
            int r = qRed(pixel);
            int g = qGreen(pixel);
            int b = qBlue(pixel);

            // 计算红色通道与其他通道的差异
            double value = r - (g + b) / 2.0;
            value = std::clamp(value, 0.0, 255.0);

            if (value > maxVal) {
                maxVal = value;
                maxPos = w;
            }
        }

        // 如果在该行检测到了激光点
        if (maxPos != -1) {
            // 估计亚像素级的 x 坐标
            double sumIntensity = 0.0;
            double weightedSum = 0.0;

            // 在激光点附近取三个像素进行亚像素估计
            for (int x = maxPos - 1; x <= maxPos + 1; ++x) {
                if (x >= x0 && x <= x1) {
                    QRgb pixel = smoothedImage.pixel(x, h);
                    int r = qRed(pixel);
                    int g = qGreen(pixel);
                    int b = qBlue(pixel);

                    double intensity = r - (g + b) / 2.0;
                    if (intensity > 0) {
                        sumIntensity += intensity;
                        weightedSum += x * intensity;
                    }
                }
            }

            double subpixelX = static_cast<double>(maxPos);
            if (sumIntensity != 0.0) {
                subpixelX = weightedSum / sumIntensity;
            }

            // 将检测到的点添加到输出向量
            laserLineCoord.append(Vector2d(subpixelX, h));
        }
    }

}

//////////////////////////////////////////////////////////////////////
void hw3::detectLaser
(QImage const& inputImage, /* input */
 QVector<Vector2d>& laserLineCoord, /* output */
 const QPolygonF& boundingQuad) {

  // - same as the previous version but using
  //   QPolygonF::containsPoint() function to decide whether or not to
  //   add the detected point to the output vector

    if (inputImage.format() != QImage::Format_RGB32)
    { //unsupported format
        return;
    }

    // 应用平滑滤波器以减少噪声
    QVector<int> filter = {1, 4, 6, 4, 1};
    QImage filteredImage = hw1::filterImage(inputImage, filter);

    // 创建一个向量来存储每行的最大强度值
    QVector<double> rowMaxValues(filteredImage.height(), 0.0);

    for (int h = 0; h < filteredImage.height(); ++h) {
        for (int w = 0; w < filteredImage.width(); ++w) {
            if (!boundingQuad.containsPoint(QPointF(w, h), Qt::OddEvenFill)) {
                continue; // 如果不在多边形范围内，跳过
            }

            QRgb pixel = filteredImage.pixel(w, h);

            int r = qRed(pixel);
            int g = qGreen(pixel);
            int b = qBlue(pixel);

            // 计算红色通道与其他通道的差异
            int v = r - (g + b) / 2;
            v = std::clamp(v, 0, 255);

            if (v > rowMaxValues[h]) {
                rowMaxValues[h] = v;
            }
        }
    }

    // 计算平均最大值，用于动态阈值
    double avgMax = 0.0;
    for (double maxVal : rowMaxValues) {
        avgMax += maxVal;
    }
    avgMax /= filteredImage.height();
    //search average maximum of each row
    //TODO ...

    // 第二步：根据动态阈值检测激光点
    for (int h = 0; h < filteredImage.height(); ++h) {
        double maxValInRow = 0.0;
        int maxIndexInRow = -1;

         if (rowMaxValues[h] < 0.8 * avgMax) {
            continue; // 如果这一行的最大值低于阈值，跳过该行
        }

        for (int w = 0; w < filteredImage.width(); ++w) {

            if (!boundingQuad.containsPoint(QPointF(w, h), Qt::OddEvenFill)) {
                continue; // 如果不在多边形范围内，跳过
            }

            QRgb pixel = filteredImage.pixel(w, h);
            int r = qRed(pixel);
            int g = qGreen(pixel);
            int b = qBlue(pixel);

            // 计算红色通道与其他通道的差异
            int value = r - (g + b) / 2;
            value = std::clamp(value, 0, 255);

            if (value > maxValInRow) {
                maxValInRow = value;
                maxIndexInRow = w;
            }
        }

        // 如果在该行检测到了激光点
        if (maxIndexInRow != -1) {
            // 估计亚像素级的 x 坐标
            double sumIntensity = 0.0;
            double weightedSum = 0.0;

            // 在激光点附近取三个像素进行亚像素估计
            for (int x = maxIndexInRow - 1; x <= maxIndexInRow + 1; ++x) {
                if (x >= 0 && x < filteredImage.width()) {
                    if (!boundingQuad.containsPoint(QPointF(x, h), Qt::OddEvenFill)) {
                        continue; // 如果不在多边形范围内，跳过
                    }

                    QRgb pixel = filteredImage.pixel(x, h);
                    int r = qRed(pixel);
                    int g = qGreen(pixel);
                    int b = qBlue(pixel);

                    double intensity = r - (g + b) / 2.0;
                    if (intensity > 0) {
                        sumIntensity += intensity;
                        weightedSum += x * intensity;
                    }
                }
            }

            double subpixelX = static_cast<double>(maxIndexInRow);
            if (sumIntensity != 0.0) {
                subpixelX = weightedSum / sumIntensity;
            }

            // 将检测到的点添加到输出向量
            laserLineCoord.append(Vector2d(subpixelX, h));
        }
    }
}

//////////////////////////////////////////////////////////////////////
void hw3::checkerboardPlane
(Matrix3d const& R, /* input */
 Vector3d const& T, /* input */
 Vector4d & checkerboardPlane /* output */) {

  // with [r0 r1 r2] = R
  //
  // the implicit equation of the plane is 
  // r2^t(p-T) = 0
  //
  // the first three coordinates of of checkerboardPlane vector must
  // be equal to r2

    // 根据 [r0 r1 r2] = R，提取 r2 列向量
    Vector3d r2 = R.col(2);

    // 计算平面方程的偏移量 d = -r2^T * T
    double d = -r2.dot(T);

    // 设置 checkerboardPlane 的前三个元素为 r2，第四个元素为 d
    checkerboardPlane.head<3>() = r2;

    checkerboardPlane(3) = d;
}

//////////////////////////////////////////////////////////////////////
void hw3::triangulate
(Matrix3d const& K, /* input */
 Vector5d const& kc, /* input */
 Vector4d& planeCoefficients,
 QVector<Vector2d> const& imagePoints, /* input */
 QVector<Vector3d> & worldPoints /* output */ ) {

  // - derived from hw2::triangulate() but all the computation is done
  //   in normalized coordinates; i.e., the world coordinate system is
  //   the camera coordinate system

    worldPoints.clear();

    Matrix3d K_inv = K.inverse();

    for (const auto& imagePoint : imagePoints) {

        Vector3d u_dist_pix(imagePoint(0), imagePoint(1), 1.0);

        Vector3d u_dist_norm = K_inv * u_dist_pix;

        double x_dist = u_dist_norm(0);
        double y_dist = u_dist_norm(1);
        double r2 = x_dist * x_dist + y_dist * y_dist;

        double k1 = kc(0);
        double k2 = kc(1);
        double p1 = kc(2);
        double p2 = kc(3);
        double k3 = kc(4);

        double x_undist = x_dist;
        double y_undist = y_dist;

        for (int iter = 0; iter < 5; ++iter) {
            double r2_undist = x_undist * x_undist + y_undist * y_undist;
            double radial_dist = 1 + k1 * r2_undist + k2 * r2_undist * r2_undist + k3 * r2_undist * r2_undist * r2_undist;
            double x_tangential = 2 * p1 * x_undist * y_undist + p2 * (r2_undist + 2 * x_undist * x_undist);
            double y_tangential = p1 * (r2_undist + 2 * y_undist * y_undist) + 2 * p2 * x_undist * y_undist;

            double x_proj = (x_dist - x_tangential) / radial_dist;
            double y_proj = (y_dist - y_tangential) / radial_dist;

            double dx = x_proj - x_undist;
            double dy = y_proj - y_undist;

            x_undist = x_proj;
            y_undist = y_proj;

            if (dx * dx + dy * dy < 1e-12)
                break;
        }

        Vector3d u_norm(x_undist, y_undist, 1.0);

        Vector3d plane_normal = planeCoefficients.head<3>();
        double plane_d = planeCoefficients(3);

        double denominator = plane_normal.dot(u_norm);
        if (std::abs(denominator) < 1e-6) {
            continue;
        }

        double lambda = -plane_d / denominator;

        Vector3d X = lambda * u_norm;

        worldPoints.push_back(X);
    }
}

//////////////////////////////////////////////////////////////////////
void hw3::estimatePlane
(QVector<Vector3d> const& laserPoints, /* input */
 Vector4d & laserPlane /* output */) {

  laserPlane = Vector4d::Zero(); // << 0,0,0,0;
  int nPoints = laserPoints.size();

  // - fit a plane to the laserPoints in the least squares sense
  //   using the SVD method
  //
  // - if pc is the centroid of the points p_i in the vector
  //   laserPoints, build a matrix A of nPoints rows by 3 columns, and
  //   fill its i-th row with (p_i-pc)^t
  //
  // - the normal vector N of the laser plane is the right hand side
  //   singular vector of the matrix A corresponding to its minimum
  //   singular value
  //
  // - the implicit equation of the plane is N^t(p-pc)=0
  //
  // - compute and print the minimum square error

  // Check if there are enough points to define a plane
  if (nPoints < 3) {
      std::cerr << "Not enough points to estimate a plane." << std::endl;
      return;
  }

  // Compute the centroid pc of the points p_i in laserPoints
  Vector3d pc = Vector3d::Zero();
  for (const auto& p : laserPoints) {
      pc += p;
  }
  pc /= nPoints;

  // Build a matrix A of nPoints rows by 3 columns
  // Fill its i-th row with (p_i - pc)^T
  MatrixXd A(nPoints, 3);
  for (int i = 0; i < nPoints; ++i) {
      Vector3d pi = laserPoints[i];
      Vector3d diff = pi - pc;
      A.row(i) = diff.transpose();
  }

  // Compute the SVD of matrix A
  // The normal vector N of the laser plane is the right singular vector
  // corresponding to the minimum singular value
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
  Vector3d N = svd.matrixV().col(2); // The last column corresponds to the smallest singular value

  // Construct the implicit equation of the plane: N^T(p - pc) = 0
  // Compute the plane offset d = -N^T * pc
  double d = -N.dot(pc);

  // Set the laserPlane output vector
  // The first three coordinates are equal to N
  laserPlane.head<3>() = N;
  laserPlane(3) = d;

  // Compute and print the minimum square error
  // The error is the sum of squared distances from the points to the plane
  double mse = 0.0;
  for (const auto& p : laserPoints) {
      double distance = N.dot(p - pc);
      mse += distance * distance;
  }
  mse /= nPoints; // Compute the mean squared error

  std::cout << "Minimum square error: " << mse << std::endl;

}

//////////////////////////////////////////////////////////////////////
void hw3::estimateTurntableCoordSystem
(QVector<Matrix3d>  const& R_pose,                /* input */
 QVector<Vector3d>  const& T_pose,                /* input */
 Vector3d                & patternCenter,         /* input/output */
 Vector3d                & turntableWorldCenter,  /* output */
 QVector<Matrix3d>       & turntableWorldRotation /* output */) {

  int nPoses = R_pose.size();
  if(nPoses< 2 || nPoses!=T_pose.size()) return;

  // [r0i r1i r2i] = R_pose[i] ... 0<=i<R_pose.size()==T_pose.size()

  // - here we assume that r0i is direction of rotation of the turntable,
  //   and it is independent of i, i.e., r0i = r0 for all i
  //
  // - patternCenter(0) contains an input value
  //   corresponding to the x coord of points on the turntable
  //   surface, as measured in the corresponding pose
  //
  // - patternCenter(1) and patternCenter(2) are output values
  //   they measure the (y,z) coordinates of the center of rotation
  // - these coordinates should be the same for all the poses
  //
  // - if p=(px,py,pz)^2 represents the center of rotation in in
  //   normalized camera coordinates, and x=patternCenter(0) is the
  //   input value, and ti=T_pose[i], then for each value of i in the
  //   range 0<=i<nPoses the following equation should hold
  //
  //   p = x*r0 + y*r1i + z*r2i + ti
  //
  //   where p,y, and z are unknown, and everything else is known
  //
  // - he previous equation can be rewritten as
  //
  //   y * r1 + z * r2 - p = -(x*r0+ti) 
  //
  // - to solve the problem, let X=(y,z,px,py,pz)^t be the vector of unknowns
  //
  // - create a matrix A of 3*nPoses rows, and 5 columns, and a vector
  //   b of dimension 3*nPoses
  //
  // - for each value of i in the range 0<=i<nPoses
  //     - fill 3 rows of A with the submatrix [r1 r2 -I], where I is
  //       the identity matrix
  //     - fill 3 rows of the vector b with the vector -(x*r0+ti)
  //
  // - the problem is reduced to solving the system of 3*nPoses linear
  //   equations in 5 variables
  //
  //   A*X = b
  //
  // - since in general we will have at least two poses, we will have
  //   more equations than variables, and the system will not have a
  //   solution
  // - instead solve the equations in the least squares sense, by
  //   minimizing \|A*X-b\|^2
  // - note that there are no constraints on X here
  //   
  // - Eigen::JacobiSVD can be used to compute the SVD decomposition of the
  //   matrix A, and to solve the problem
  //
  // - finally, since the rotation [r0 r1i r2i] = R_pose[i] in the
  //   vector R_pose, all have the the vector r0 pointing down, and
  //   the vectors r1i and r2i spanning a plane paralel to the
  //   turntable top surface, you need to generate for each value of i
  //   a new rotaion matrix R_tt_i = [ r2i r1i -r0 ], and add it in
  //   the output vector turntableWorldRotation
  //
  // - this matrix R_tt_i, together with the translation vector
  //   R_tt=turntableWorldCenter define the pose of the world
  //   (turntable ) coordinate system for the image corresponding to
  //   the index i
  //
  // - not that in this coordinate system, the first two rows of the
  //   rotation matrix span the turntable top plane, and the third
  //   axis points up
  // **Step 1: Extract common rotation axis r0**
  // Assume that r0i is the same for all i, i.e., r0i = r0
  Vector3d r0 = R_pose[0].col(0);

  // **Step 2: Get the known x coordinate from patternCenter**
  double x = patternCenter(0);

  // **Step 3: Initialize matrix A and vector b**
  // A is of size (3*nPoses) x 5
  // b is of size 3*nPoses
  MatrixXd A(3 * nPoses, 5);
  VectorXd b(3 * nPoses);

  // **Step 4: Fill in A and b for each pose**
  for (int i = 0; i < nPoses; ++i) {
      // Extract rotation vectors r1i and r2i from R_pose[i]
      Vector3d r1i = R_pose[i].col(1);
      Vector3d r2i = R_pose[i].col(2);
      Vector3d ti = T_pose[i];

      // Fill in the submatrix [r1i r2i -I] in A
      A.block<3, 1>(3 * i, 0) = r1i;
      A.block<3, 1>(3 * i, 1) = r2i;
      A.block<3, 3>(3 * i, 2) = -Matrix3d::Identity();

      // Compute b_i = -(x * r0 + ti)
      Vector3d b_i = -(x * r0 + ti);

      // Fill in b
      b.segment<3>(3 * i) = b_i;
  }

  // **Step 5: Solve the least squares problem A * X = b**
  // X = [y, z, px, py, pz]^T
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  VectorXd X = svd.solve(b);

  // **Step 6: Extract the unknowns y, z, and p from X**
  double y = X(0);
  double z = X(1);
  Vector3d p = X.segment<3>(2);

  // **Step 7: Set the output values**
  // Update patternCenter with the computed y and z
  patternCenter(1) = y;
  patternCenter(2) = z;

  // Set the turntable world center
  turntableWorldCenter = p;

  // **Step 8: Generate turntable world rotations for each pose**
  turntableWorldRotation.clear();
  for (int i = 0; i < nPoses; ++i) {
      Vector3d r1i = R_pose[i].col(1);
      Vector3d r2i = R_pose[i].col(2);

      // Construct R_tt_i = [r2i, r1i, -r0]
      Matrix3d R_tt_i;
      R_tt_i.col(0) = r2i;
      R_tt_i.col(1) = r1i;
      R_tt_i.col(2) = -r0;

      // Add R_tt_i to the output vector
      turntableWorldRotation.push_back(R_tt_i);
  }

}

//////////////////////////////////////////////////////////////////////
void hw3::generateWireframeModel
(const double height, const double radius, const int sides,
 /* outputs */
 QVector<Vector3d>& worldCoord, QVector<int>& edge) {
  
  worldCoord.clear();
  edge.clear();

  if(height<=0.0 || radius<=0.0 || sides<3) return;

  // - to test the turntable coordinate system computed with the
  //   previous function you have to generate a wireframe gemetric
  //   model, which the application will render on the images, in a
  //   primitive augmented reality fashion
  //
  // - the wireframe model is defined by the vector of vertex coordinates 
  //   worldCoord, and by a number of edges
  //
  // - each vertex of the model has an index in the range
  //   0<=indx<worldCoord.size() - an edge is a unordered pair of
  //   different vertex indices (i,j) (edges (i,j) and (j,i) describe
  //   the same geometric entity) representing a straight line segment
  //   connecting vertices i and j in 3 space
  //
  // - the vertex coordinates are defined in the world (turntable)
  //   coordinate system
  //
  // - the index pairs (i,j) are stored as consecutive int values in
  //   the edge vector; as a result, the size of the output edge
  //   vector is equal to two times the number of edges
  //
  // - the application will project the vertex coordinates onto the
  //   image plane, using the pose of the turntable corresponding to
  //   the image being shown, and for each edge it will draw a line
  //   segment connecting the corresponding projected vertices
  //
  // - with the given parameters, you can build, for example, a
  //   polygonal cylinder of the given number of sides, radius and height
  //
  // - but you could build something else as a function of the given
  //   parameters as well
  //
  // - a vertex with z coordinate equal to 0 showld be rendered as
  //   laying on the turntable top plane; positive z is above the
  //   plane, and negative z is below the plane

  // Constants for calculations
  const double PI = 3.14159265358979323846;
  double angleIncrement = 2.0 * PI / sides;

  // Generate vertices
  // Indices for vertices
  int baseIndex = 0;
  int topIndex = sides;

  // Generate base vertices (z = 0)
  for (int i = 0; i < sides; ++i) {
      double theta = i * angleIncrement;
      double x = radius * cos(theta);
      double y = radius * sin(theta);
      double z = 0.0; // Base at z = 0

      worldCoord.push_back(Vector3d(x, y, z));
  }

  // Generate top vertices (z = height)
  for (int i = 0; i < sides; ++i) {
      double theta = i * angleIncrement;
      double x = radius * cos(theta);
      double y = radius * sin(theta);
      double z = height; // Top at z = height

      worldCoord.push_back(Vector3d(x, y, z));
  }

  // Generate edges
  // Base edges
  for (int i = 0; i < sides; ++i) {
      int current = baseIndex + i;
      int next = baseIndex + (i + 1) % sides;

      // Edge between current and next vertex on base
      edge.push_back(current);
      edge.push_back(next);
  }

  // Top edges
  for (int i = 0; i < sides; ++i) {
      int current = topIndex + i;
      int next = topIndex + (i + 1) % sides;

      // Edge between current and next vertex on top
      edge.push_back(current);
      edge.push_back(next);
  }

  // Side edges
  for (int i = 0; i < sides; ++i) {
      int baseVertex = baseIndex + i;
      int topVertex = topIndex + i;

      // Edge between base vertex and corresponding top vertex
      edge.push_back(baseVertex);
      edge.push_back(topVertex);
  }
}
