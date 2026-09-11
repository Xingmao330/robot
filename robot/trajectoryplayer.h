#ifndef TRAJECTORYPLAYER_H
#define TRAJECTORYPLAYER_H

#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <QVector>

class TrajectoryPlayer final : public QObject
{
    Q_OBJECT

public:
    explicit TrajectoryPlayer(QObject *parent = nullptr);
    void addWaypoint(const QVector<float> &jointAngles);
    void clear();
    void start();
    void pause();
    void stop();
    [[nodiscard]] int waypointCount() const;
    [[nodiscard]] bool isPlaying() const;
    [[nodiscard]] bool isPaused() const;

signals:
    void jointAnglesChanged(const QVector<float> &jointAngles);
    void playbackStateChanged(bool playing, bool paused);

private slots:
    void advanceFrame();

private:
    QTimer m_timer;
    QElapsedTimer m_clock;
    QVector<QVector<float>> m_waypoints;
    int m_segmentIndex = 0;
    int m_segmentElapsedMs = 0;
    int m_segmentDurationMs = 900;
    bool m_playing = false;
    bool m_paused = false;
};

#endif // TRAJECTORYPLAYER_H
