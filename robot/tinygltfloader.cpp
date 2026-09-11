#include "tinygltfloader.h"
#include "glbmodel.h"
#include "../tinygltf/tiny_gltf_v3.h"
#include <QFile>
#include <QFileInfo>

#include <algorithm>
#include <limits>

namespace {
    const unsigned char *accessorData(const tg3_model *model, const tg3_accessor &accessor) {
        if (accessor.buffer_view < 0 || accessor.buffer_view >= int(model->buffer_views_count)) return nullptr;
        const tg3_buffer_view &view = model->buffer_views[accessor.buffer_view];
        if (view.buffer < 0 || view.buffer >= int(model->buffers_count)) return nullptr;
        const tg3_buffer &buffer = model->buffers[view.buffer];
        const uint64_t offset = view.byte_offset + accessor.byte_offset;
        return offset < buffer.data.count ? buffer.data.data + offset : nullptr;
    }

    int attributeAccessor(const tg3_primitive &primitive, const char *name) {
        for (uint32_t i = 0; i < primitive.attributes_count; ++i)
            if (tg3_str_equals_cstr(primitive.attributes[i].key, name)) return primitive.attributes[i].value;
        return -1;
    }

    QString parserErrors(const tinygltf3::ErrorStack &errors) {
        QStringList messages;
        for (uint32_t i = 0; i < errors.count(); ++i) messages << QString::fromUtf8(errors.entry(i)->message);
        return messages.join('\n');
    }
}

bool TinyGltfLoader::load(const QString &fileName, GlbModel *model, QString *errorMessage) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        *errorMessage = file.errorString();
        return false;
    }
    const QByteArray data = file.readAll(), baseDir = QFileInfo(fileName).absolutePath().toUtf8();
    tinygltf3::Model source;
    tinygltf3::ErrorStack errors;
    if (tg3_parse_glb(source.get(), errors.get(), reinterpret_cast<const uint8_t *>(data.constData()),
                      uint64_t(data.size()), baseDir.constData(), uint32_t(baseDir.size()), nullptr) != TG3_OK) {
        *errorMessage = parserErrors(errors);
        return false;
    }
    const tg3_model *src = source.get();
    GlbModel loaded;
    QVector<QVector<int> > meshPrimitives(int(src->meshes_count));
    for (uint32_t meshId = 0; meshId < src->meshes_count; ++meshId) {
        const tg3_mesh &srcMesh = src->meshes[meshId];
        for (uint32_t primitiveId = 0; primitiveId < srcMesh.primitives_count; ++primitiveId) {
            const tg3_primitive &primitive = srcMesh.primitives[primitiveId];
            const int positionId = attributeAccessor(primitive, "POSITION"), normalId = attributeAccessor(
                primitive, "NORMAL");
            if (positionId < 0 || normalId < 0 || primitive.indices < 0 || positionId >= int(src->accessors_count) ||
                normalId >= int(src->accessors_count) || primitive.indices >= int(src->accessors_count)) {
                *errorMessage = QObject::tr("网格 primitive 缺少 POSITION、NORMAL 或索引");
                return false;
            }
            const tg3_accessor &positions = src->accessors[positionId], &normals = src->accessors[normalId], &indices =
                    src->accessors[primitive.indices];
            const unsigned char *positionData = accessorData(src, positions), *normalData = accessorData(src, normals),
                    *indexData = accessorData(src, indices);
            if (!positionData || !normalData || !indexData || positions.component_type != TG3_COMPONENT_TYPE_FLOAT ||
                normals.component_type != TG3_COMPONENT_TYPE_FLOAT || positions.type != TG3_TYPE_VEC3 || normals.type !=
                TG3_TYPE_VEC3 || positions.count != normals.count) {
                *errorMessage = QObject::tr("仅支持包含 FLOAT VEC3 法线的网格");
                return false;
            }
            const int positionStride = tg3_accessor_byte_stride(&positions, &src->buffer_views[positions.buffer_view]);
            const int normalStride = tg3_accessor_byte_stride(&normals, &src->buffer_views[normals.buffer_view]);
            const int indexStride = tg3_accessor_byte_stride(&indices, &src->buffer_views[indices.buffer_view]);
            if (positionStride <= 0 || normalStride <= 0 || indexStride <= 0) {
                *errorMessage = QObject::tr("无效的 primitive 步长");
                return false;
            }
            GlbMesh mesh;
            mesh.vertices.reserve(int(positions.count) * 6);
            mesh.indices.reserve(int(indices.count));
            mesh.minimum = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            mesh.maximum = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
            if (primitive.material >= 0 && primitive.material < int(src->materials_count)) {
                const auto &material = src->materials[primitive.material].pbr_metallic_roughness;
                mesh.baseColor = {
                    float(material.base_color_factor[0]), float(material.base_color_factor[1]),
                    float(material.base_color_factor[2])
                };
                mesh.metallic = float(material.metallic_factor);
                mesh.roughness = float(material.roughness_factor);
            }
            for (uint64_t i = 0; i < positions.count; ++i) {
                const float *p = reinterpret_cast<const float *>(positionData + i * uint64_t(positionStride));
                const float *n = reinterpret_cast<const float *>(normalData + i * uint64_t(normalStride));
                mesh.vertices << p[0] << p[1] << p[2] << n[0] << n[1] << n[2];
                mesh.minimum.setX(std::min(mesh.minimum.x(), p[0])); mesh.minimum.setY(std::min(mesh.minimum.y(), p[1])); mesh.minimum.setZ(std::min(mesh.minimum.z(), p[2]));
                mesh.maximum.setX(std::max(mesh.maximum.x(), p[0])); mesh.maximum.setY(std::max(mesh.maximum.y(), p[1])); mesh.maximum.setZ(std::max(mesh.maximum.z(), p[2]));
            }
            for (uint64_t i = 0; i < indices.count; ++i) {
                const unsigned char *v = indexData + i * uint64_t(indexStride);
                if (indices.component_type == TG3_COMPONENT_TYPE_UNSIGNED_SHORT)
                    mesh.indices << *reinterpret_cast<const quint16 *>(v);
                else if (indices.component_type == TG3_COMPONENT_TYPE_UNSIGNED_INT)
                    mesh.indices << *reinterpret_cast<const quint32 *>(v);
                else if (indices.component_type == TG3_COMPONENT_TYPE_UNSIGNED_BYTE) mesh.indices << *v;
                else {
                    *errorMessage = QObject::tr("不支持的 primitive 索引类型");
                    return false;
                }
            }
            meshPrimitives[int(meshId)].append(loaded.meshData.size());
            loaded.meshData.append(std::move(mesh));
        }
    }
    loaded.nodeData.resize(int(src->nodes_count));
    for (uint32_t i = 0; i < src->nodes_count; ++i) {
        const tg3_node &node = src->nodes[i];
        QMatrix4x4 transform;
        if (node.has_matrix) {
            float values[16];
            for (int j = 0; j < 16; ++j) values[j] = float(node.matrix[j]);
            transform = QMatrix4x4(values);
        } else {
            transform.translate(float(node.translation[0]), float(node.translation[1]), float(node.translation[2]));
            transform.rotate(QQuaternion(float(node.rotation[3]), float(node.rotation[0]), float(node.rotation[1]),
                                         float(node.rotation[2])));
            transform.scale(float(node.scale[0]), float(node.scale[1]), float(node.scale[2]));
        }
        const QVector<int> meshes = node.mesh >= 0 && node.mesh < int(src->meshes_count)
                                        ? meshPrimitives[node.mesh]
                                        : QVector<int>{};
        loaded.nodeData[int(i)] = {QString::fromUtf8(node.name.data, int(node.name.len)), meshes, -1, transform};
    }
    for (uint32_t parent = 0; parent < src->nodes_count; ++parent)
        for (uint32_t child = 0; child < src->nodes[parent].children_count; ++child)
            loaded.nodeData[src->nodes[parent].children[child]].parentIndex = int(parent);
    *model = std::move(loaded);
    return true;
}
