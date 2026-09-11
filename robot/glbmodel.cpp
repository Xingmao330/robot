#include "glbmodel.h"

bool GlbModel::isEmpty() const { return meshData.isEmpty() || nodeData.isEmpty(); }
const QVector<GlbMesh> &GlbModel::meshes() const { return meshData; }
const QVector<GlbNode> &GlbModel::nodes() const { return nodeData; }
