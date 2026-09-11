#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <QVector>
#include <QVector3D>

class RobotModel;

class IKSolver
{
public:
    struct Result {
        bool reached = false;
        float positionError = 0.0f;
        QVector<float> jointAngles;
    };

    Result solvePosition(RobotModel &model, const QVector3D &targetPosition) const;

    int maximumIterations = 48;
    float damping = 0.12f;
    float positionTolerance = 0.005f;
};

#endif // IKSOLVER_H
