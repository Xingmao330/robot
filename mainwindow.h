#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "robot/forwardkinematics.h"
#include "robot/jointanimator.h"
#include "robot/iksolver.h"
#include "robot/robotmodel.h"

class RobotOpenGLWidget;

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
    ForwardKinematics m_forwardKinematics;
    JointAnimator m_jointAnimator;
    IKSolver m_ikSolver;
};
#endif // MAINWINDOW_H
