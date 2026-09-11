#include "iksolver.h"

#include "kinematicstate.h"
#include "robotmodel.h"

#include <QtMath>

#include <algorithm>
#include <cmath>

namespace {

QVector3D solveSymmetric3x3(float a00, float a01, float a02, float a11, float a12, float a22,
                             const QVector3D &b)
{
    const float c00 = a11 * a22 - a12 * a12;
    const float c01 = a02 * a12 - a01 * a22;
    const float c02 = a01 * a12 - a02 * a11;
    const float c11 = a00 * a22 - a02 * a02;
    const float c12 = a01 * a02 - a00 * a12;
    const float c22 = a00 * a11 - a01 * a01;
    const float determinant = a00 * c00 + a01 * c01 + a02 * c02;
    if (std::abs(determinant) < 1.0e-8f) return {};
    const float inverseDeterminant = 1.0f / determinant;
    return {(c00 * b.x() + c01 * b.y() + c02 * b.z()) * inverseDeterminant,
            (c01 * b.x() + c11 * b.y() + c12 * b.z()) * inverseDeterminant,
            (c02 * b.x() + c12 * b.y() + c22 * b.z()) * inverseDeterminant};
}

} // namespace

IKSolver::Result IKSolver::solvePosition(RobotModel &model, const QVector3D &targetPosition) const
{
    Result result;
    const QVector<float> initialAngles = model.jointAngles();
    for (int iteration = 0; iteration < maximumIterations; ++iteration) {
        const KinematicState state = model.kinematicState();
        if (!state.valid || state.jointAxes.isEmpty()) break;
        const QVector3D error = targetPosition - state.endEffectorPosition;
        result.positionError = error.length();
        if (result.positionError <= positionTolerance) { result.reached = true; break; }

        QVector<QVector3D> jacobian;
        float a00 = damping * damping, a01 = 0.0f, a02 = 0.0f;
        float a11 = damping * damping, a12 = 0.0f, a22 = damping * damping;
        for (int index = 0; index < state.jointAxes.size(); ++index) {
            const QVector3D column = QVector3D::crossProduct(state.jointAxes[index], state.endEffectorPosition - state.jointPositions[index]);
            jacobian.append(column);
            a00 += column.x() * column.x(); a01 += column.x() * column.y(); a02 += column.x() * column.z();
            a11 += column.y() * column.y(); a12 += column.y() * column.z(); a22 += column.z() * column.z();
        }
        const QVector3D projectedError = solveSymmetric3x3(a00, a01, a02, a11, a12, a22, error);
        for (int index = 0; index < jacobian.size(); ++index) {
            const float deltaRadians = QVector3D::dotProduct(jacobian[index], projectedError);
            const float deltaDegrees = std::clamp(qRadiansToDegrees(deltaRadians), -4.0f, 4.0f);
            model.setJointAngle(index, model.jointAngles()[index] + deltaDegrees);
        }
    }
    const KinematicState finalState = model.kinematicState();
    if (finalState.valid) {
        result.positionError = (targetPosition - finalState.endEffectorPosition).length();
        result.reached = result.positionError <= positionTolerance;
    }
    result.jointAngles = model.jointAngles();
    for (int index = 0; index < initialAngles.size(); ++index) model.setJointAngle(index, initialAngles[index]);
    return result;
}
