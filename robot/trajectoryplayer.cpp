#include "trajectoryplayer.h"

#include <algorithm>

TrajectoryPlayer::TrajectoryPlayer(QObject *parent)
    : QObject(parent)
{
    m_timer.setInterval(16);
    connect(&m_timer, &QTimer::timeout, this, &TrajectoryPlayer::advanceFrame);
}

void TrajectoryPlayer::addWaypoint(const QVector<float> &jointAngles)
{
    if (!jointAngles.isEmpty()) m_waypoints.append(jointAngles);
}

void TrajectoryPlayer::clear()
{
    stop();
    m_waypoints.clear();
}

void TrajectoryPlayer::start()
{
    if (m_waypoints.size() < 2) return;
    if (m_playing && m_paused) {
        m_paused = false;
        m_clock.restart();
        m_timer.start();
        emit playbackStateChanged(true, false);
        return;
    }
    if (m_playing) return;
    m_segmentIndex = 0;
    m_segmentElapsedMs = 0;
    m_playing = true;
    m_paused = false;
    emit jointAnglesChanged(m_waypoints.first());
    m_clock.restart();
    m_timer.start();
    emit playbackStateChanged(true, false);
}

void TrajectoryPlayer::pause()
{
    if (!m_playing || m_paused) return;
    m_timer.stop();
    m_paused = true;
    emit playbackStateChanged(true, true);
}

void TrajectoryPlayer::stop()
{
    if (!m_playing) return;
    m_timer.stop();
    m_playing = false;
    m_paused = false;
    emit playbackStateChanged(false, false);
}

int TrajectoryPlayer::waypointCount() const { return m_waypoints.size(); }
bool TrajectoryPlayer::isPlaying() const { return m_playing; }
bool TrajectoryPlayer::isPaused() const { return m_paused; }

void TrajectoryPlayer::advanceFrame()
{
    m_segmentElapsedMs += int(m_clock.restart());
    const float progress = std::min(float(m_segmentElapsedMs) / float(m_segmentDurationMs), 1.0f);
    const float eased = progress * progress * progress * (progress * (progress * 6.0f - 15.0f) + 10.0f);
    const QVector<float> &from = m_waypoints[m_segmentIndex];
    // 最后一段插值回第一个点，闭合轨迹后继续循环，避免跳变。
    const QVector<float> &to = m_waypoints[(m_segmentIndex + 1) % m_waypoints.size()];
    QVector<float> angles;
    angles.reserve(std::min(from.size(), to.size()));
    for (int index = 0; index < from.size() && index < to.size(); ++index)
        angles.append(from[index] + (to[index] - from[index]) * eased);
    emit jointAnglesChanged(angles);
    if (progress >= 1.0f) {
        m_segmentIndex = (m_segmentIndex + 1) % m_waypoints.size();
        m_segmentElapsedMs = 0;
    }
}
