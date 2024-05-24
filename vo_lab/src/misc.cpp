#include <Eigen/Eigen>

Eigen::Vector3d fitLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    // ax+by+c = 0
    // a(x1-x2)+b(y1-y2)=0
    // so: a=y1-y2, b=x2-x1, c=-(y1-y2)x1-(x2-x1)y1=y2x1-x2y1
    Eigen::Vector3d line;
    line.x() = p1.y() - p2.y();
    line.y() = p2.x() - p1.x();
    line.z() = p2.y() * p1.x() - p2.x() * p1.y();
    if (fabs(line.z()) > 1e-7) {
        line.x() /= line.z();
        line.y() /= line.z();
        line.z() = 1;
    }
    return line;
}

double disToLine(const Eigen::Vector3d& line, const Eigen::Vector2d& p) {
    double a = line.transpose() * p.homogeneous();
    double b = line.head<2>().norm();
    return a / b;
}

Eigen::Vector3d crossMidLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    Eigen::Vector3d line = fitLine(p1, p2);
    Eigen::Vector3d cross_line;
    cross_line.x() = line.y();
    cross_line.y() = -line.x();
    Eigen::Vector2d mid = (p1 + p2) / 2;
    cross_line.z() = -cross_line.head<2>().transpose() * mid;
    if (fabs(cross_line.z()) > 1e-7) {
        cross_line.x() /= cross_line.z();
        cross_line.y() /= cross_line.z();
        cross_line.z() = 1;
    }
    return cross_line;
}

Eigen::Vector3d fitCircle(const Eigen::Vector2d& p1,
                          const Eigen::Vector2d& p2,
                          const Eigen::Vector2d& p3) {
    Eigen::Vector3d circle;
    auto line1 = crossMidLine(p1, p2);
    auto line2 = crossMidLine(p2, p3);

    if (fabs(line1.x() * line2.y() - line1.y() * line2.x()) < 1e-7) {
        return circle;
    }

    circle.y() = -(line2.x() * line1.z() - line1.x() * line2.z()) /
                 (line2.x() * line1.y() - line1.x() * line2.y());
    circle.x() = -(line2.y() * line1.z() - line1.y() * line2.z()) /
                 (line2.y() * line1.x() - line1.y() * line2.x());
    circle.z() = (circle.head<2>() - p1).norm();
    return circle;
}

Eigen::Vector4d fitPlane(const Eigen::Vector3d& p1,
                         const Eigen::Vector3d& p2,
                         const Eigen::Vector3d& p3) {
    auto l1 = p1 - p2;
    auto l2 = p2 - p3;
    auto n = l1.cross(l2);
    double d = -n.transpose() * p1;
    if (fabs(d) > 1e-7) {
        n /= d;
        d = 1;
    }
    Eigen::Vector4d p;
    p.head<3>() = n;
    p[3] = d;
    return p;
}

double disToPlane(const Eigen::Vector4d& plane, const Eigen::Vector3d& p) {
    double a = plane.transpose() * p.homogeneous();
    double b = plane.head<3>().norm();
    return a / b;
}