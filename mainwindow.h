#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>

#include "robot/jointanimator.h"
#include "robot/iksolver.h"
#include "robot/trajectoryplayer.h"
#include "robot/robotmodel.h"

class RobotOpenGLWidget;
class QSlider;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    void updateRobotView();

    RobotOpenGLWidget *m_robotView = nullptr;
    RobotModel m_robotModel;
    JointAnimator m_jointAnimator;
    IKSolver m_ikSolver;
    TrajectoryPlayer m_trajectoryPlayer;
    QVector<QSlider *> m_jointSliders;
};
#endif // MAINWINDOW_H
