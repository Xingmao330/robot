#ifndef KINEMATICSTATE_H
#define KINEMATICSTATE_H

#include <QVector>
#include <QVector3D>

struct KinematicState
{
    QVector<QVector3D> jointPositions;
    QVector<QVector3D> jointAxes;
    QVector3D endEffectorPosition;
    bool valid = false;
};

#endif // KINEMATICSTATE_H
