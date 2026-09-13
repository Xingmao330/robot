#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "joint.h"
#include "glbmodel.h"
#include "kinematicstate.h"

#include <QVector>

class RobotModel
{
public:
    RobotModel();

    void setJointAngle(int index, float degrees);
    bool loadGlbFile(const QString &fileName, QString *errorMessage);
    [[nodiscard]] const QVector<Joint> &joints() const;
    [[nodiscard]] bool hasGlbModel() const;
    [[nodiscard]] const GlbModel &glbModel() const;
    [[nodiscard]] QVector<QMatrix4x4> glbNodeTransforms() const;
    [[nodiscard]] QVector<float> jointAngles() const;
    [[nodiscard]] KinematicState kinematicState() const;

private:
    QVector<Joint> m_joints;
    GlbModel m_glbModel;
};

#endif // ROBOTMODEL_H
