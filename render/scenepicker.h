#ifndef SCENEPICKER_H
#define SCENEPICKER_H

#include <QMatrix4x4>
#include <QPoint>
#include <QSize>
#include <QVector>

class GlbModel;

class ScenePicker
{
public:
    static int pickNode(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                        const QMatrix4x4 &view, const GlbModel &model,
                        const QVector<QMatrix4x4> &nodeTransforms);
};

#endif // SCENEPICKER_H
