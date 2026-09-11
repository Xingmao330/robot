#ifndef GLBMODEL_H
#define GLBMODEL_H

#include <QMatrix4x4>
#include <QString>
#include <QVector>

struct GlbMesh
{
    QVector<float> vertices; // position + normal, six floats per vertex
    QVector<quint32> indices;
    QVector3D minimum;
    QVector3D maximum;
    QVector3D baseColor {0.8f, 0.8f, 0.8f};
    float metallic = 0.0f;
    float roughness = 0.7f;
};

struct GlbNode
{
    QString name;
    QVector<int> meshIndices;
    int parentIndex = -1;
    QMatrix4x4 localTransform;
};

class GlbModel
{
public:
    [[nodiscard]] bool isEmpty() const;
    [[nodiscard]] const QVector<GlbMesh> &meshes() const;
    [[nodiscard]] const QVector<GlbNode> &nodes() const;

    QVector<GlbMesh> meshData;
    QVector<GlbNode> nodeData;
};

#endif // GLBMODEL_H
