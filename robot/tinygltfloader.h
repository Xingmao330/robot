#ifndef TINYGLTFLOADER_H
#define TINYGLTFLOADER_H

#include <QString>

class GlbModel;

class TinyGltfLoader
{
public:
    static bool load(const QString &fileName, GlbModel *model, QString *errorMessage);
};

#endif // TINYGLTFLOADER_H
