#include "robotmodel.h"

#include "tinygltfloader.h"

#include <functional>

namespace {

QMatrix4x4 transform(float tx, float ty, float tz, float sx, float sy, float sz)
{
    QMatrix4x4 result;
    result.translate(tx, ty, tz);
    result.scale(sx, sy, sz);
    return result;
}

QMatrix4x4 translation(float x, float y, float z)
{
    QMatrix4x4 result;
    result.translate(x, y, z);
    return result;
}

} // namespace

RobotModel::RobotModel()
{
    // links[0] 是固定底座；后续每个 Link 分别由同下标 Joint 驱动。
    // 无模型时的测试数据，伪3关节机械臂
    m_links = {
        {QStringLiteral("base"), transform(0.0f, 0.25f, 0.0f, 1.1f, 0.5f, 1.1f)},
        {QStringLiteral("shoulder"), transform(0.0f, 0.28f, 0.0f, 0.55f, 0.56f, 0.55f)},
        {QStringLiteral("upper_arm"), transform(0.0f, 0.9f, 0.0f, 0.32f, 1.8f, 0.32f)},
        {QStringLiteral("forearm"), transform(0.0f, 0.75f, 0.0f, 0.26f, 1.5f, 0.26f)},
    };
    m_joints = {
        {QStringLiteral("J1"), {0.0f, 1.0f, 0.0f}, translation(0.0f, 0.5f, 0.0f)},
        {QStringLiteral("J2"), {0.0f, 0.0f, 1.0f}, translation(0.0f, 0.55f, 0.0f)},
        {QStringLiteral("J3"), {0.0f, 0.0f, 1.0f}, translation(0.0f, 1.8f, 0.0f)},
    };
}

void RobotModel::setJointAngle(int index, float degrees)
{
    if (index >= 0 && index < m_joints.size())
        m_joints[index].setAngleDegrees(degrees);
}

const QVector<Joint> &RobotModel::joints() const
{
    return m_joints;
}

const QVector<Link> &RobotModel::links() const
{
    return m_links;
}

bool RobotModel::loadGlbFile(const QString &fileName, QString *errorMessage)
{
    GlbModel loaded;
    if (!TinyGltfLoader::load(fileName, &loaded, errorMessage))
        return false;
    m_glbModel = std::move(loaded);
    m_joints.clear();
    // IRB4600 的 J4/J6 是腕部滚转轴，须沿本地 X（连杆纵向）旋转；
    // 若使用 Y 轴会表现为腕部“摆动”。
    const QVector3D axes[] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f},
                              {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
    QMatrix4x4 identity;
    for (int index = 0; index < 6; ++index)
        m_joints.append(Joint(QStringLiteral("J%1").arg(index + 1), axes[index], identity));
    return true;
}

bool RobotModel::hasGlbModel() const
{
    return !m_glbModel.isEmpty();
}

const GlbModel &RobotModel::glbModel() const
{
    return m_glbModel;
}

QVector<QMatrix4x4> RobotModel::glbNodeTransforms() const
{
    const QVector<GlbNode> &nodes = m_glbModel.nodes();
    QVector<QMatrix4x4> result(nodes.size());
    QVector<bool> resolved(nodes.size(), false);
    std::function<QMatrix4x4(int)> resolve = [&](int index) -> QMatrix4x4 {
        if (resolved[index]) return result[index];
        QMatrix4x4 local = nodes[index].localTransform;
        const QString &name = nodes[index].name;
        if (name.startsWith(QStringLiteral("link"))) {
            bool isNumber = false; const int jointIndex = name.mid(4).toInt(&isNumber) - 1;
            if (isNumber && jointIndex >= 0 && jointIndex < m_joints.size())
                local.rotate(m_joints[jointIndex].angleDegrees(), m_joints[jointIndex].axis());
        }
        result[index] = nodes[index].parentIndex < 0 ? local : resolve(nodes[index].parentIndex) * local;
        resolved[index] = true; return result[index];
    };
    QMatrix4x4 modelUnitScale;
    // 源文件以毫米保存；GLB 根节点已有 0.1 缩放，此处再转为米制视图单位。
    modelUnitScale.scale(0.01f);
    for (int index = 0; index < nodes.size(); ++index) result[index] = modelUnitScale * resolve(index);
    return result;
}

QVector<float> RobotModel::jointAngles() const
{
    QVector<float> angles;
    angles.reserve(m_joints.size());
    for (const Joint &joint : m_joints) angles.append(joint.angleDegrees());
    return angles;
}

KinematicState RobotModel::kinematicState() const
{
    KinematicState state;
    if (!hasGlbModel()) return state;
    const QVector<GlbNode> &nodes = m_glbModel.nodes();
    QVector<QMatrix4x4> worldFrames(nodes.size());
    QVector<bool> resolved(nodes.size(), false);
    state.jointPositions.resize(m_joints.size());
    state.jointAxes.resize(m_joints.size());
    std::function<QMatrix4x4(int)> resolve = [&](int index) -> QMatrix4x4 {
        if (resolved[index]) return worldFrames[index];
        const GlbNode &node = nodes[index];
        const QMatrix4x4 parent = node.parentIndex < 0 ? QMatrix4x4() : resolve(node.parentIndex);
        QMatrix4x4 local = node.localTransform;
        const QString &name = node.name;
        if (name.startsWith(QStringLiteral("link"))) {
            bool isNumber = false;
            const int jointIndex = name.mid(4).toInt(&isNumber) - 1;
            if (isNumber && jointIndex >= 0 && jointIndex < m_joints.size()) {
                // 旋转前的节点坐标系就是该关节帧：记录真实的世界枢轴和轴方向。
                const QMatrix4x4 jointFrame = parent * local;
                state.jointPositions[jointIndex] = jointFrame.map({0.0f, 0.0f, 0.0f}) * 0.01f;
                state.jointAxes[jointIndex] = jointFrame.mapVector(m_joints[jointIndex].axis()).normalized();
                local.rotate(m_joints[jointIndex].angleDegrees(), m_joints[jointIndex].axis());
            }
        }
        worldFrames[index] = parent * local;
        resolved[index] = true;
        return worldFrames[index];
    };
    for (int index = 0; index < nodes.size(); ++index) resolve(index);
    for (int index = 0; index < nodes.size(); ++index) {
        if (nodes[index].name == QStringLiteral("link6")) {
            state.endEffectorPosition = worldFrames[index].map({0.0f, 0.0f, 0.0f}) * 0.01f;
            state.valid = true;
            break;
        }
    }
    return state;
}
