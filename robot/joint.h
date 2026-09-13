#ifndef JOINT_H
#define JOINT_H

#include <QString>
#include <QVector3D>

class Joint
{
public:
    Joint(QString name, QVector3D axis, float minimumDegrees = -180.0f, float maximumDegrees = 180.0f);

    [[nodiscard]] const QString &name() const;
    [[nodiscard]] float angleDegrees() const;
    [[nodiscard]] const QVector3D &axis() const;
    void setAngleDegrees(float degrees);

private:
    QString m_name;
    QVector3D m_axis;
    float m_angleDegrees = 0.0f;
    float m_minimumDegrees;
    float m_maximumDegrees;
};

#endif // JOINT_H
