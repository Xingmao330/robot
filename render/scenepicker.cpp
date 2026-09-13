#include "scenepicker.h"

#include "../robot/glbmodel.h"

#include <algorithm>
#include <limits>

namespace {
    bool intersectsBox(const QVector3D &origin, const QVector3D &direction, const QVector3D &minimum,
                       const QVector3D &maximum, float *distance) {
        float nearDistance = 0.0f, farDistance = std::numeric_limits<float>::max();
        for (int axis = 0; axis < 3; ++axis) {
            if (qFuzzyIsNull(direction[axis])) {
                if (origin[axis] < minimum[axis] || origin[axis] > maximum[axis]) return false;
                continue;
            }
            float first = (minimum[axis] - origin[axis]) / direction[axis];
            float second = (maximum[axis] - origin[axis]) / direction[axis];
            if (first > second) std::swap(first, second);
            nearDistance = std::max(nearDistance, first);
            farDistance = std::min(farDistance, second);
            if (nearDistance > farDistance) return false;
        }
        *distance = nearDistance;
        return farDistance >= 0.0f;
    }
}

int ScenePicker::pickNode(const QPoint &screenPosition, const QSize &viewport, const QMatrix4x4 &projection,
                          const QMatrix4x4 &view, const GlbModel &model,
                          const QVector<QMatrix4x4> &nodeTransforms) {
    if (viewport.width() <= 0 || viewport.height() <= 0) return -1;
    const float x = 2.0f * float(screenPosition.x()) / float(viewport.width()) - 1.0f;
    const float y = 1.0f - 2.0f * float(screenPosition.y()) / float(viewport.height());
    bool invertible = false;
    const QMatrix4x4 inverseVP = (projection * view).inverted(&invertible);
    if (!invertible) return -1;
    const QVector3D nearPoint = inverseVP.map({x, y, -1.0f});
    const QVector3D direction = (inverseVP.map({x, y, 1.0f}) - nearPoint).normalized();
    float closestDistance = std::numeric_limits<float>::max();
    int closestNode = -1;
    const QVector<GlbNode> &nodes = model.nodes();
    for (int nodeIndex = 0; nodeIndex < nodes.size() && nodeIndex < nodeTransforms.size(); ++nodeIndex) {
        bool invertibleModel = false;
        const QMatrix4x4 inverseModel = nodeTransforms[nodeIndex].inverted(&invertibleModel);
        if (!invertibleModel) continue;
        const QVector3D localOrigin = inverseModel.map(nearPoint), localDirection = inverseModel.mapVector(direction);
        for (const int meshIndex: nodes[nodeIndex].meshIndices) {
            if (meshIndex < 0 || meshIndex >= model.meshes().size()) continue;
            float distance = 0.0f;
            const GlbMesh &mesh = model.meshes()[meshIndex];
            if (intersectsBox(localOrigin, localDirection, mesh.minimum, mesh.maximum, &distance) && distance <
                closestDistance) {
                closestDistance = distance;
                closestNode = nodeIndex;
            }
        }
    }
    return closestNode;
}
