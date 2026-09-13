#include "mainwindow.h"

#include "render/robotopenglwidget.h"

#include <QCoreApplication>
#include <QDockWidget>
#include <QFile>
#include <QFormLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QStatusBar>
#include <QStringList>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QDir>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("机械臂可视化"));
    resize(1100, 760);
    m_robotView = new RobotOpenGLWidget(this);
    setCentralWidget(m_robotView);
    connect(m_robotView, &RobotOpenGLWidget::componentSelected, this, [this](const QString &name, int index) {
        statusBar()->showMessage(index >= 0 ? tr("已选中：%1（节点 %2）").arg(name).arg(index)
                                            : tr("未选中任何机械臂组件"));
    });

    QString modelError;
    const QString modelFile = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("irb4600.glb"));
    const QString sourceModelFile = QDir::current().filePath(QStringLiteral("irb4600.glb"));
    const bool modelLoaded = m_robotModel.loadGlbFile(QFile::exists(sourceModelFile) ? sourceModelFile : modelFile, &modelError);
    if (modelLoaded)
        m_robotView->setGlbModel(m_robotModel.glbModel());
    m_jointAnimator.setJointCount(m_robotModel.joints().size());
    connect(&m_jointAnimator, &JointAnimator::jointAngleChanged, this, [this](int index, float degrees) {
        m_robotModel.setJointAngle(index, degrees);
        updateRobotView();
    });
    connect(&m_trajectoryPlayer, &TrajectoryPlayer::jointAnglesChanged, this, [this](const QVector<float> &angles) {
        for (int index = 0; index < angles.size(); ++index) {
            m_robotModel.setJointAngle(index, angles[index]);
            if (index < m_jointSliders.size()) {
                const QSignalBlocker blocker(m_jointSliders[index]);
                m_jointSliders[index]->setValue(qRound(angles[index]));
            }
        }
        updateRobotView();
    });
    connect(m_robotView, &RobotOpenGLWidget::ikTargetMoved, this, [this](const QVector3D &targetPosition) {
        m_trajectoryPlayer.stop();
        const IKSolver::Result result = m_ikSolver.solvePosition(m_robotModel, targetPosition);
        for (int index = 0; index < result.jointAngles.size(); ++index)
            m_robotModel.setJointAngle(index, result.jointAngles[index]);
        updateRobotView();
    });

    auto *jointDock = new QDockWidget(tr("关节控制"), this);
    jointDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    auto *controlWidget = new QWidget(jointDock);
    auto *layout = new QVBoxLayout(controlWidget);
    layout->addWidget(new QLabel(tr("角度单位：°"), controlWidget));

    const int jointCount = m_robotModel.joints().size();
    for (int index = 0; index < jointCount; ++index) {
        auto *slider = new QSlider(Qt::Horizontal, controlWidget);
        slider->setRange(-180, 180);
        slider->setValue(0);
        slider->setTickPosition(QSlider::TicksBelow);
        slider->setTickInterval(45);
        m_jointSliders.append(slider);
        auto *row = new QFormLayout;
        row->addRow(tr("J%1").arg(index + 1), slider);
        layout->addLayout(row);
        connect(slider, &QSlider::valueChanged, this, [this, index](int degrees) {
            m_trajectoryPlayer.stop();
            m_robotModel.setJointAngle(index, float(degrees));
            updateRobotView();
        });
    }

    auto *zeroButton = new QPushButton(tr("关节回零"), controlWidget);
    connect(zeroButton, &QPushButton::clicked, controlWidget, [controlWidget] {
        for (QSlider *slider : controlWidget->findChildren<QSlider *>())
            slider->setValue(0);
    });
    layout->addWidget(zeroButton);

    layout->addWidget(new QLabel(tr("轨迹记录"), controlWidget));
    auto *waypointList = new QListWidget(controlWidget);
    waypointList->setToolTip(tr("按记录时保存当前六轴角度；播放按记录顺序经过各点"));
    layout->addWidget(waypointList);
    auto *recordButton = new QPushButton(tr("记录当前位置"), controlWidget);
    connect(recordButton, &QPushButton::clicked, this, [this, waypointList] {
        const QVector<float> angles = m_robotModel.jointAngles();
        m_trajectoryPlayer.addWaypoint(angles);
        QStringList values;
        for (int index = 0; index < angles.size(); ++index)
            values.append(tr("J%1=%2°").arg(index + 1).arg(angles[index], 0, 'f', 0));
        waypointList->addItem(tr("点 %1：%2").arg(m_trajectoryPlayer.waypointCount()).arg(values.join(QStringLiteral("  "))));
        statusBar()->showMessage(tr("已记录轨迹点 %1").arg(m_trajectoryPlayer.waypointCount()));
    });
    layout->addWidget(recordButton);
    auto *startButton = new QPushButton(tr("开始播放"), controlWidget);
    connect(startButton, &QPushButton::clicked, this, [this] {
        if (m_trajectoryPlayer.waypointCount() < 2) {
            statusBar()->showMessage(tr("请至少记录两个轨迹点"));
            return;
        }
        if (m_trajectoryPlayer.isPlaying() && !m_trajectoryPlayer.isPaused()) {
            m_trajectoryPlayer.pause();
            statusBar()->showMessage(tr("轨迹播放已暂停"));
            return;
        }
        m_trajectoryPlayer.start();
        statusBar()->showMessage(tr("正在循环播放 %1 个轨迹点").arg(m_trajectoryPlayer.waypointCount()));
    });
    connect(&m_trajectoryPlayer, &TrajectoryPlayer::playbackStateChanged, startButton,
            [startButton](bool playing, bool paused) {
        startButton->setText(playing && !paused ? QObject::tr("暂停") : QObject::tr("开始播放"));
    });
    layout->addWidget(startButton);
    auto *clearButton = new QPushButton(tr("清除记录"), controlWidget);
    connect(clearButton, &QPushButton::clicked, this, [this, waypointList] {
        m_trajectoryPlayer.clear();
        waypointList->clear();
        statusBar()->showMessage(tr("轨迹记录已清除"));
    });
    layout->addWidget(clearButton);

    layout->addStretch();
    jointDock->setWidget(controlWidget);
    addDockWidget(Qt::LeftDockWidgetArea, jointDock);

    auto *toolbar = addToolBar(tr("视图"));
    QAction *resetViewAction = toolbar->addAction(tr("重置视角"));
    connect(resetViewAction, &QAction::triggered, m_robotView, &RobotOpenGLWidget::resetCamera);
    if (!modelLoaded) {
        statusBar()->showMessage(tr("模型加载失败：%1").arg(modelError));
    } else {
        statusBar()->showMessage(tr("左键平移  |  中键绕目标旋转  |  滚轮缩放"));
    }
    updateRobotView();
}

MainWindow::~MainWindow() = default;

void MainWindow::updateRobotView()
{
    if (m_robotModel.hasGlbModel())
        m_robotView->setGlbNodeTransforms(m_robotModel.glbNodeTransforms());
}
