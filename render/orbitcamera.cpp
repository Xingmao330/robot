#include "orbitcamera.h"

#include <QtMath>
#include <algorithm>
#include <cmath>

QVector3D OrbitCamera::position() const
{
    const float pitch = qDegreesToRadians(m_pitchDegrees), yaw = qDegreesToRadians(m_yawDegrees);
    return m_target + QVector3D(m_distance * std::cos(pitch) * std::cos(yaw), m_distance * std::sin(-pitch), m_distance * std::cos(pitch) * std::sin(yaw));
}
QMatrix4x4 OrbitCamera::viewMatrix() const
{
    QMatrix4x4 view; view.lookAt(position(), m_target, {0.0f, 1.0f, 0.0f}); return view;
}
void OrbitCamera::orbit(float deltaX, float deltaY)
{
    m_yawDegrees += deltaX * 0.45f;
    m_pitchDegrees = std::clamp(m_pitchDegrees + deltaY * 0.45f, -89.0f, 89.0f);
}
void OrbitCamera::pan(float deltaX, float deltaY)
{
    const QVector3D direction = forward();
    const QVector3D right = QVector3D::crossProduct(direction, {0.0f, 1.0f, 0.0f}).normalized();
    const QVector3D up = QVector3D::crossProduct(right, direction).normalized();
    const float scale = m_distance * 0.0018f;
    m_target -= right * deltaX * scale;
    m_target += up * deltaY * scale;
}
void OrbitCamera::zoom(float wheelSteps) { m_distance = std::clamp(m_distance * std::pow(0.85f, wheelSteps), 1.0f, 50.0f); }
void OrbitCamera::reset() { m_target = {0.0f, 1.2f, 0.0f}; m_yawDegrees = -42.0f; m_pitchDegrees = -22.0f; m_distance = 7.0f; }
QVector3D OrbitCamera::forward() const { return (m_target - position()).normalized(); }
