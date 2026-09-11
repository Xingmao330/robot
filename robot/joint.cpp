#include "joint.h"

#include <algorithm>

Joint::Joint(QString name, QVector3D axis, QMatrix4x4 origin, float minimumDegrees, float maximumDegrees)
    : m_name(std::move(name))
    , m_axis(axis.normalized())
    , m_origin(std::move(origin))
    , m_minimumDegrees(minimumDegrees)
    , m_maximumDegrees(maximumDegrees)
{
}

const QString &Joint::name() const
{
    return m_name;
}

float Joint::angleDegrees() const
{
    return m_angleDegrees;
}

const QVector3D &Joint::axis() const
{
    return m_axis;
}

void Joint::setAngleDegrees(float degrees)
{
    m_angleDegrees = std::clamp(degrees, m_minimumDegrees, m_maximumDegrees);
}

QMatrix4x4 Joint::localTransform() const
{
    QMatrix4x4 transform = m_origin;
    transform.rotate(m_angleDegrees, m_axis);
    return transform;
}
