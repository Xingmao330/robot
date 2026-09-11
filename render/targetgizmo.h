#ifndef TARGETGIZMO_H
#define TARGETGIZMO_H

#include <QMatrix4x4>
#include <QPoint>
#include <QSize>
#include <QVector3D>

class TargetGizmo
{
public:
    void setPosition(const QVector3D &position);
    void setAxesFromTransform(const QMatrix4x4 &transform);
    [[nodiscard]] const QVector3D &position() const;
    [[nodiscard]] QVector3D axisDirection(int axisIndex) const;
    [[nodiscard]] bool isVisible() const;
    [[nodiscard]] float axisLength(const QVector3D &cameraPosition) const;
    bool beginDrag(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                   const QMatrix4x4 &view, const QVector3D &cameraPosition);
    bool drag(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
              const QMatrix4x4 &view);
    void endDrag();

private:
    enum class Axis { None, X, Y, Z, ViewPlane };
    bool rayFromScreen(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                       const QMatrix4x4 &view, QVector3D *origin, QVector3D *direction) const;
    [[nodiscard]] QVector3D axisVector(Axis axis) const;

    QVector3D m_position;
    QVector3D m_axisX {1.0f, 0.0f, 0.0f};
    QVector3D m_axisY {0.0f, 1.0f, 0.0f};
    QVector3D m_axisZ {0.0f, 0.0f, 1.0f};
    QVector3D m_dragStartPosition;
    QVector3D m_dragStartPlanePoint;
    QVector3D m_dragPlaneNormal;
    float m_dragStartAxisDistance = 0.0f;
    Axis m_activeAxis = Axis::None;
    bool m_visible = false;
    bool m_dragging = false;
};

#endif // TARGETGIZMO_H
