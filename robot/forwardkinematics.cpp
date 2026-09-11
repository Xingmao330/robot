#include "forwardkinematics.h"

#include "robotmodel.h"

QVector<QMatrix4x4> ForwardKinematics::solve(const RobotModel &model) const
{
    const QVector<Link> &links = model.links();
    const QVector<Joint> &joints = model.joints();
    QVector<QMatrix4x4> worldTransforms;
    worldTransforms.reserve(links.size());
    if (links.isEmpty())
        return worldTransforms;

    // 固定底座不受关节影响。之后沿关节链逐步累乘父变换。
    worldTransforms.append(links[0].visualTransform());
    QMatrix4x4 parentTransform;
    const int movableLinkCount = std::min(joints.size(), links.size() - 1);
    for (int index = 0; index < movableLinkCount; ++index) {
        parentTransform *= joints[index].localTransform();
        worldTransforms.append(parentTransform * links[index + 1].visualTransform());
    }
    return worldTransforms;
}
