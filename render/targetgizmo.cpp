#include "targetgizmo.h"

#include <QLineF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QPointF projectPoint(const QVector3D &position, const QSize &viewport, const QMatrix4x4 &projection, const QMatrix4x4 &view)
{
    const QVector4D clip = projection * view * QVector4D(position, 1.0f);
    const QVector3D ndc = clip.toVector3DAffine();
    return {(ndc.x() + 1.0f) * viewport.width() * 0.5f, (1.0f - ndc.y()) * viewport.height() * 0.5f};
}

float pointToSegmentDistance(const QPointF &point, const QPointF &start, const QPointF &end)
{
    const QPointF line = end - start;
    const float lengthSquared = float(QPointF::dotProduct(line, line));
    if (qFuzzyIsNull(lengthSquared)) return QLineF(point, start).length();
    const float factor = std::clamp(float(QPointF::dotProduct(point - start, line)) / lengthSquared, 0.0f, 1.0f);
    return QLineF(point, start + line * factor).length();
}

float rayAxisDistance(const QVector3D &rayOrigin, const QVector3D &rayDirection, const QVector3D &axisOrigin,
                      const QVector3D &axis, float *axisDistance)
{
    const QVector3D offset = rayOrigin - axisOrigin;
    const float directionAxis = QVector3D::dotProduct(rayDirection, axis);
    const float rayOffset = QVector3D::dotProduct(rayDirection, offset);
    const float axisOffset = QVector3D::dotProduct(axis, offset);
    const float denominator = 1.0f - directionAxis * directionAxis;
    if (std::abs(denominator) < 1.0e-5f) return std::numeric_limits<float>::max();
    *axisDistance = (axisOffset - directionAxis * rayOffset) / denominator;
    const float rayDistance = directionAxis * *axisDistance - rayOffset;
    return (rayOrigin + rayDirection * rayDistance - (axisOrigin + axis * *axisDistance)).length();
}
} // namespace

void TargetGizmo::setPosition(const QVector3D &position) { m_position = position; m_visible = true; }
const QVector3D &TargetGizmo::position() const { return m_position; }
bool TargetGizmo::isVisible() const { return m_visible; }
float TargetGizmo::axisLength(const QVector3D &cameraPosition) const { return std::clamp((cameraPosition - m_position).length() * 0.12f, 0.10f, 0.55f); }

bool TargetGizmo::beginDrag(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                             const QMatrix4x4 &view, const QVector3D &cameraPosition)
{
    if (!m_visible) return false;
    const float length = axisLength(cameraPosition); const QPointF cursor = screenPosition;
    Axis closestAxis = Axis::None; float closestDistance = 12.0f;
    for (const Axis axis : {Axis::X, Axis::Y, Axis::Z}) {
        const float distance = pointToSegmentDistance(cursor, projectPoint(m_position, viewport, projection, view), projectPoint(m_position + axisVector(axis) * length, viewport, projection, view));
        if (distance < closestDistance) { closestDistance = distance; closestAxis = axis; }
    }
    QVector3D rayOrigin, rayDirection;
    if (closestAxis == Axis::None || !rayFromScreen(screenPosition, viewport, projection, view, &rayOrigin, &rayDirection)) return false;
    m_activeAxis = closestAxis; m_dragStartPosition = m_position;
    rayAxisDistance(rayOrigin, rayDirection, m_position, axisVector(m_activeAxis), &m_dragStartAxisDistance);
    m_dragging = true;
    return true;
}

bool TargetGizmo::drag(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection, const QMatrix4x4 &view)
{
    if (!m_dragging) return false;
    QVector3D rayOrigin, rayDirection;
    if (!rayFromScreen(screenPosition, viewport, projection, view, &rayOrigin, &rayDirection)) return false;
    float currentAxisDistance = 0.0f;
    if (!std::isfinite(rayAxisDistance(rayOrigin, rayDirection, m_dragStartPosition, axisVector(m_activeAxis), &currentAxisDistance))) return false;
    m_position = m_dragStartPosition + axisVector(m_activeAxis) * (currentAxisDistance - m_dragStartAxisDistance);
    return true;
}

void TargetGizmo::endDrag() { m_dragging = false; m_activeAxis = Axis::None; }

bool TargetGizmo::rayFromScreen(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                                 const QMatrix4x4 &view, QVector3D *origin, QVector3D *direction) const
{
    if (viewport.width() <= 0 || viewport.height() <= 0) return false;
    const float x = 2.0f * float(screenPosition.x()) / float(viewport.width()) - 1.0f;
    const float y = 1.0f - 2.0f * float(screenPosition.y()) / float(viewport.height());
    bool invertible = false; const QMatrix4x4 inverse = (projection * view).inverted(&invertible);
    if (!invertible) return false;
    *origin = inverse.map({x, y, -1.0f}); *direction = (inverse.map({x, y, 1.0f}) - *origin).normalized();
    return true;
}

QVector3D TargetGizmo::axisVector(Axis axis)
{
    if (axis == Axis::X) return {1.0f, 0.0f, 0.0f};
    if (axis == Axis::Y) return {0.0f, 1.0f, 0.0f};
    return {0.0f, 0.0f, 1.0f};
}
