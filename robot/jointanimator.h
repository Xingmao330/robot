#ifndef JOINTANIMATOR_H
#define JOINTANIMATOR_H

#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <QVector>

class JointAnimator final : public QObject
{
    Q_OBJECT

public:
    explicit JointAnimator(QObject *parent = nullptr);
    void setJointCount(int count);
    void animateTo(int jointIndex, float targetDegrees);
    void setDurationMs(int durationMs);
    void setMotionLimits(float maximumVelocityDegPerSecond, float maximumAccelerationDegPerSecondSquared);

signals:
    void jointAngleChanged(int jointIndex, float currentDegrees);

private slots:
    void advanceFrame();

private:
    QTimer m_timer;
    QElapsedTimer m_clock;
    QVector<float> m_current;
    QVector<float> m_start;
    QVector<float> m_target;
    QVector<int> m_elapsedMs;
    QVector<int> m_durationMs;
    int m_minimumDurationMs = 120;
    float m_maximumVelocityDegPerSecond = 180.0f;
    float m_maximumAccelerationDegPerSecondSquared = 720.0f;
};

#endif // JOINTANIMATOR_H
