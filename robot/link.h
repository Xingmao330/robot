#ifndef LINK_H
#define LINK_H

#include <QMatrix4x4>
#include <QString>

class Link
{
public:
    Link(QString name, QMatrix4x4 visualTransform);

    [[nodiscard]] const QString &name() const;
    [[nodiscard]] const QMatrix4x4 &visualTransform() const;

private:
    QString m_name;
    QMatrix4x4 m_visualTransform;
};

#endif // LINK_H
