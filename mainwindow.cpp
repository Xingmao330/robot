#include "mainwindow.h"

#include "render/robotopenglwidget.h"

#include <QCoreApplication>
#include <QDockWidget>
#include <QFile>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QStatusBar>
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
    connect(m_robotView, &RobotOpenGLWidget::ikTargetMoved, this, [this](const QVector3D &targetPosition) {
        const IKSolver::Result result = m_ikSolver.solvePosition(m_robotModel, targetPosition);
        for (int index = 0; index < result.jointAngles.size(); ++index)
            m_jointAnimator.animateTo(index, result.jointAngles[index]);
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
        auto *row = new QFormLayout;
        row->addRow(tr("J%1").arg(index + 1), slider);
        layout->addLayout(row);
        connect(slider, &QSlider::valueChanged, this, [this, index](int degrees) {
            m_jointAnimator.animateTo(index, float(degrees));
        });
    }

    auto *zeroButton = new QPushButton(tr("关节回零"), controlWidget);
    connect(zeroButton, &QPushButton::clicked, controlWidget, [controlWidget] {
        for (QSlider *slider : controlWidget->findChildren<QSlider *>())
            slider->setValue(0);
    });
    layout->addWidget(zeroButton);
    auto *ikButton = new QPushButton(tr("IK 测试：TCP 前移"), controlWidget);
    ikButton->setToolTip(tr("以当前 link6 TCP 为起点，求解向 X 方向前移 100 mm 的关节目标"));
    connect(ikButton, &QPushButton::clicked, this, [this] {
        const KinematicState currentState = m_robotModel.kinematicState();
        if (!currentState.valid) { statusBar()->showMessage(tr("IK 不可用：未找到 link6 TCP")); return; }
        const QVector3D targetPosition = currentState.endEffectorPosition + QVector3D(0.10f, 0.0f, 0.0f);
        m_robotView->setIkTargetPosition(targetPosition);
        const IKSolver::Result result = m_ikSolver.solvePosition(m_robotModel, targetPosition);
        for (int index = 0; index < result.jointAngles.size(); ++index)
            m_jointAnimator.animateTo(index, result.jointAngles[index]);
        statusBar()->showMessage(result.reached ? tr("IK 已收敛，位置误差 %1 mm").arg(result.positionError * 1000.0f, 0, 'f', 1)
                                                : tr("IK 未完全收敛，当前误差 %1 mm").arg(result.positionError * 1000.0f, 0, 'f', 1));
    });
    layout->addWidget(ikButton);
    layout->addStretch();
    jointDock->setWidget(controlWidget);
    addDockWidget(Qt::LeftDockWidgetArea, jointDock);

    auto *toolbar = addToolBar(tr("视图"));
    QAction *resetViewAction = toolbar->addAction(tr("重置视角"));
    connect(resetViewAction, &QAction::triggered, m_robotView, &RobotOpenGLWidget::resetCamera);
    if (!modelLoaded) {
        statusBar()->showMessage(tr("模型加载失败：%1；正在显示原型机械臂").arg(modelError));
    }
    updateRobotView();
    statusBar()->showMessage(tr("左键平移  |  中键绕目标旋转  |  滚轮缩放"));
}

MainWindow::~MainWindow() = default;

void MainWindow::updateRobotView()
{
    if (m_robotModel.hasGlbModel())
        m_robotView->setGlbNodeTransforms(m_robotModel.glbNodeTransforms());
    else
        m_robotView->setLinkTransforms(m_forwardKinematics.solve(m_robotModel));
}
