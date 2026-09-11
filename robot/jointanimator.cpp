#include "jointanimator.h"

#include <algorithm>
#include <cmath>

JointAnimator::JointAnimator(QObject *parent)
    : QObject(parent)
{
    m_timer.setInterval(16);
    connect(&m_timer, &QTimer::timeout, this, &JointAnimator::advanceFrame);
}

void JointAnimator::setJointCount(int count)
{
    count = std::max(count, 0);
    m_current.fill(0.0f, count);
    m_start = m_current;
    m_target = m_current;
    m_elapsedMs.fill(0, count);
    m_durationMs.fill(m_minimumDurationMs, count);
    m_timer.stop();
}

void JointAnimator::animateTo(int jointIndex, float targetDegrees)
{
    if (jointIndex < 0 || jointIndex >= m_current.size())
        return;
    m_start[jointIndex] = m_current[jointIndex];
    m_target[jointIndex] = targetDegrees;
    m_elapsedMs[jointIndex] = 0;
    const float distance = std::abs(m_target[jointIndex] - m_start[jointIndex]);
    // 五次 S 曲线的峰值速度约为 1.875 * distance / duration，
    // 峰值加速度约为 5.8 * distance / duration²。按两项上限取较长时长。
    const float velocityTime = 1.875f * distance / m_maximumVelocityDegPerSecond;
    const float accelerationTime = std::sqrt(5.8f * distance / m_maximumAccelerationDegPerSecondSquared);
    m_durationMs[jointIndex] = std::max(m_minimumDurationMs, int(std::ceil(1000.0f * std::max(velocityTime, accelerationTime))));
    if (!m_timer.isActive()) {
        m_clock.restart();
        m_timer.start();
    }
}

void JointAnimator::setDurationMs(int durationMs)
{
    m_minimumDurationMs = std::max(durationMs, 1);
}

void JointAnimator::setMotionLimits(float maximumVelocityDegPerSecond, float maximumAccelerationDegPerSecondSquared)
{
    m_maximumVelocityDegPerSecond = std::max(maximumVelocityDegPerSecond, 1.0f);
    m_maximumAccelerationDegPerSecondSquared = std::max(maximumAccelerationDegPerSecondSquared, 1.0f);
}

void JointAnimator::advanceFrame()
{
    const int deltaMs = int(m_clock.restart());
    bool anyActive = false;
    for (int index = 0; index < m_current.size(); ++index) {
        if (qFuzzyCompare(m_current[index] + 1.0f, m_target[index] + 1.0f))
            continue;
        m_elapsedMs[index] = std::min(m_elapsedMs[index] + deltaMs, m_durationMs[index]);
        const float progress = float(m_elapsedMs[index]) / float(m_durationMs[index]);
        // 五次 S 曲线：位置、速度与加速度在起止点均连续，接近工业伺服轨迹。
        const float eased = progress * progress * progress * (progress * (progress * 6.0f - 15.0f) + 10.0f);
        m_current[index] = m_start[index] + (m_target[index] - m_start[index]) * eased;
        emit jointAngleChanged(index, m_current[index]);
        anyActive |= progress < 1.0f;
    }
    if (!anyActive)
        m_timer.stop();
}
