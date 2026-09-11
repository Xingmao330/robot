#ifndef ROBOTOPENGLWIDGET_H
#define ROBOTOPENGLWIDGET_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>
#include <QVector>

#include <array>
#include <memory>

#include "orbitcamera.h"
#include "scenepicker.h"
#include "targetgizmo.h"

class GlbModel;

class QMouseEvent;
class QWheelEvent;

class RobotOpenGLWidget final : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit RobotOpenGLWidget(QWidget *parent = nullptr);
    ~RobotOpenGLWidget() override;

    void setLinkTransforms(const QVector<QMatrix4x4> &linkTransforms);
    void setGlbModel(const GlbModel &model);
    void setGlbNodeTransforms(const QVector<QMatrix4x4> &nodeTransforms);
    void setIkTargetPosition(const QVector3D &position);
    void resetCamera();

signals:
    void componentSelected(const QString &nodeName, int nodeIndex);
    void ikTargetMoved(const QVector3D &position);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void createSceneGeometry();
    void createGizmoResources();
    void drawTargetGizmo();
    void uploadGlbModel();
    void clearGlbResources();
    void drawGeometry(QOpenGLVertexArrayObject &vao, int count, unsigned int primitive);
    void drawCube(const QMatrix4x4 &model, const QMatrix4x4 &view);

    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_lineVao;
    QOpenGLBuffer m_lineBuffer {QOpenGLBuffer::VertexBuffer};
    int m_lineVertexCount = 0;
    QOpenGLVertexArrayObject m_cubeVao;
    QOpenGLBuffer m_cubeBuffer {QOpenGLBuffer::VertexBuffer};
    int m_cubeVertexCount = 0;
    QOpenGLVertexArrayObject m_gizmoVao;
    QOpenGLBuffer m_gizmoBuffer {QOpenGLBuffer::VertexBuffer};
    QMatrix4x4 m_projection;

    QPoint m_lastMousePosition;
    QPoint m_leftPressPosition;
    bool m_leftDragActive = false;
    bool m_draggingIkTarget = false;
    OrbitCamera m_camera;
    QVector<QMatrix4x4> m_linkTransforms;
    struct GpuMesh;
    std::vector<std::unique_ptr<GpuMesh>> m_glbMeshes;
    const GlbModel *m_glbModel = nullptr;
    QVector<QMatrix4x4> m_glbNodeTransforms;
    int m_selectedNodeIndex = -1;
    TargetGizmo m_targetGizmo;
    bool m_gizmoFollowsSelectedNode = false;
    QVector3D m_gizmoLocalOffset;
};

#endif // ROBOTOPENGLWIDGET_H
