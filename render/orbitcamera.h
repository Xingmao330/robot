#ifndef ORBITCAMERA_H
#define ORBITCAMERA_H

#include <QMatrix4x4>
#include <QVector3D>

class OrbitCamera
{
public:
    [[nodiscard]] QMatrix4x4 viewMatrix() const;
    [[nodiscard]] QVector3D position() const;
    void orbit(float deltaX, float deltaY);
    void pan(float deltaX, float deltaY);
    void zoom(float wheelSteps);
    void reset();
private:
    [[nodiscard]] QVector3D forward() const;
    QVector3D m_target {0.0f, 1.2f, 0.0f};
    float m_yawDegrees = -42.0f;
    float m_pitchDegrees = -22.0f;
    float m_distance = 7.0f;
};

#endif // ORBITCAMERA_H
