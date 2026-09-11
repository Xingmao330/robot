#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include <QMatrix4x4>
#include <QVector>

class RobotModel;

class ForwardKinematics
{
public:
    [[nodiscard]] QVector<QMatrix4x4> solve(const RobotModel &model) const;
};

#endif // FORWARDKINEMATICS_H
