#include "link.h"

Link::Link(QString name, QMatrix4x4 visualTransform)
    : m_name(std::move(name))
    , m_visualTransform(std::move(visualTransform))
{
}

const QString &Link::name() const
{
    return m_name;
}

const QMatrix4x4 &Link::visualTransform() const
{
    return m_visualTransform;
}
