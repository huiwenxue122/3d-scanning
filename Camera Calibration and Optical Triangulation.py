#include <Eigen/Dense>
#include <QtCore/QVector>
#include <QImage>
#include <QtGui/QColor>

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector5d;

double hw2::reprojectPoints(
    Matrix3d const& K, Vector5d const& kc,
    Matrix3d const& R, Vector3d const& T,
    QVector<Vector3d> const& worldPoints,
    QVector<Vector2d> const& imagePoints,
    QVector<Vector2d> & reprojectedPoints)
{
    reprojectedPoints.clear();
    reprojectedPoints.reserve(worldPoints.size());

    const double k1 = kc(0), k2 = kc(1), p1 = kc(2), p2 = kc(3), k3 = kc(4);

    double sum_sq = 0.0;
    const int n = worldPoints.size();

    for (int i = 0; i < n; ++i) {
        const Vector3d& Pw = worldPoints[i];

        // world -> camera
        Vector3d Pc = R * Pw + T;                  // (Xc, Yc, Zc)
        const double X = Pc(0), Y = Pc(1), Z = Pc(2);

        // camera -> normalized plane
        const double xn = X / Z;
        const double yn = Y / Z;

        // lens distortion (Bouguet model)
        const double r2 = xn * xn + yn * yn;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

        double xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
        double yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;

        // normalized -> pixel (K = [fx s cx; 0 fy cy; 0 0 1])
        const double fx = K(0,0);
        const double s  = K(0,1);
        const double cx = K(0,2);
        const double fy = K(1,1);
        const double cy = K(1,2);

        Vector2d u;
        u(0) = fx * xd + s * yd + cx;
        u(1) = fy * yd + cy;

        reprojectedPoints.push_back(u);

        // accumulate squared error to ground-truth 2D point
        const Vector2d& v = imagePoints[i];
        sum_sq += (u - v).squaredNorm();
    }

    // RMSE defined in the handout: sqrt( (1/(2N)) * sum ||pi - p̂i||^2 )
    double rmse = std::sqrt(sum_sq / (2.0 * std::max(1, n)));
    return rmse;
}

QVector<Vector3d> hw2::triangulate(
    Matrix3d const& K, Vector5d const& /*kc*/,
    Matrix3d const& R, Vector3d const& T,
    Vector4d const& laserPlane, double turntableAngle,
    QImage image)
{
    // 1) 检测激光线（来自作业1）
    QImage detectionImage = hw1::detectLaser(image);

    QVector<Vector3d> points;
    points.reserve(detectionImage.width() * detectionImage.height() / 20); // 预估

    const Matrix3d Kinv = K.inverse();
    const Eigen::Vector3d n3 = laserPlane.head<3>(); // (a,b,c)
    const double d = laserPlane(3);                   // 常数项

    // 撤销转盘旋转的矩阵 Rz(-α)
    const double ca = std::cos(-turntableAngle);
    const double sa = std::sin(-turntableAngle);
    Matrix3d Rz;
    Rz << ca, -sa, 0,
          sa,  ca, 0,
           0,   0, 1;

    for (int y = 0; y < detectionImage.height(); ++y) {
        const QRgb* scan = reinterpret_cast<const QRgb*>(detectionImage.constScanLine(y));
        for (int x = 0; x < detectionImage.width(); ++x) {
            // 仅处理被激光点亮的像素
            const int value = qRed(scan[x]);
            if (value == 0) continue;

            // 2) pixel -> camera ray (忽略畸变)
            Eigen::Vector3d u_pix(double(x), double(y), 1.0);
            Eigen::Vector3d dir = Kinv * u_pix;   // ≈ [xn, yn, 1]
            // （可不单位化；若想单位化： dir.normalize(); ）

            // 3) ray-plane intersection: X(λ) = λ * dir;  n·X + d = 0
            const double denom = n3.dot(dir);
            if (std::abs(denom) < 1e-12) continue; // 平行，跳过
            const double lambda = -d / denom;
            if (lambda <= 0) continue;             // 在相机后方或无效
            Vector3d p_cam = lambda * dir;

            // 4) camera -> world (转到转盘/世界坐标)
            Vector3d p_world = R.transpose() * (p_cam - T);

            // 5) 撤销该帧的转盘角
            Vector3d p_unrot = Rz * p_world;

            points.push_back(p_unrot);
        }
    }

    return points;
}
