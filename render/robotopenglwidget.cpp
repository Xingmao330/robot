#include "robotopenglwidget.h"

#include "../robot/glbmodel.h"

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QtMath>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr char vertexShaderSource[] = R"(
#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 aNormalOrColor;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
out vec3 vNormalOrColor;
out vec3 worldPosition;
void main()
{
    vec4 world = model * vec4(position, 1.0);
    gl_Position = projection * view * world;
    worldPosition = world.xyz;
    vNormalOrColor = mat3(transpose(inverse(model))) * aNormalOrColor;
}
)";

constexpr char fragmentShaderSource[] = R"(
#version 330 core
in vec3 vNormalOrColor;
in vec3 worldPosition;
out vec4 fragmentColor;
uniform bool useLighting;
uniform vec3 baseColor;
uniform float metallic;
uniform float roughness;
uniform vec3 cameraPosition;
uniform bool selected;
void main()
{
    if (!useLighting) {
        fragmentColor = vec4(vNormalOrColor, 1.0);
        return;
    }
    vec3 normal = normalize(vNormalOrColor);
    vec3 lightDirection = normalize(vec3(-0.45, 0.8, 0.35));
    vec3 viewDirection = normalize(cameraPosition - worldPosition);
    vec3 halfDirection = normalize(lightDirection + viewDirection);
    float nDotL = max(dot(normal, lightDirection), 0.0);
    float highlightExponent = mix(96.0, 8.0, clamp(roughness, 0.0, 1.0));
    float specular = pow(max(dot(normal, halfDirection), 0.0), highlightExponent);
    vec3 f0 = mix(vec3(0.04), baseColor, clamp(metallic, 0.0, 1.0));
    // 用半球环境光模拟 Blender 视口的工作室补光：金属也能反射明亮环境，
    // 因此白色机械臂不会在深色背景中变成近黑色。
    float skyAmount = 0.5 + 0.5 * normal.y;
    vec3 environment = mix(vec3(0.22), vec3(0.78), skyAmount);
    vec3 ambient = baseColor * environment * mix(0.48, 0.92, clamp(metallic, 0.0, 1.0));
    vec3 diffuse = baseColor * (1.0 - metallic) * nDotL;
    vec3 lit = ambient + (diffuse + f0 * specular * 1.6) * vec3(1.25, 1.20, 1.12);
    if (selected) lit = mix(lit, vec3(1.0, 0.55, 0.05), 0.30);
    // ACES 近似色调映射和 sRGB gamma，避免线性色彩直接输出造成的灰暗。
    lit *= 1.15;
    lit = clamp((lit * (2.51 * lit + 0.03)) / (lit * (2.43 * lit + 0.59) + 0.14), 0.0, 1.0);
    fragmentColor = vec4(pow(lit, vec3(1.0 / 2.2)), 1.0);
}
)";

struct Vertex {
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
};

} // namespace

struct RobotOpenGLWidget::GpuMesh {
    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vertices {QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer indices {QOpenGLBuffer::IndexBuffer};
    int indexCount = 0;
};

RobotOpenGLWidget::RobotOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

RobotOpenGLWidget::~RobotOpenGLWidget()
{
    makeCurrent();
    m_lineBuffer.destroy();
    m_lineVao.destroy();
    m_cubeBuffer.destroy();
    m_cubeVao.destroy();
    m_gizmoBuffer.destroy();
    m_gizmoVao.destroy();
    clearGlbResources();
    m_program.removeAllShaders();
    doneCurrent();
}

void RobotOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glLineWidth(1.0f);

    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource)
        || !m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource)
        || !m_program.link()) {
        qWarning("OpenGL shader initialization failed: %s", qPrintable(m_program.log()));
        return;
    }
    createSceneGeometry();
    createGizmoResources();
    if (m_glbModel)
        uploadGlbModel();
}

void RobotOpenGLWidget::resizeGL(int width, int height)
{
    const int safeHeight = std::max(height, 1);
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, float(width) / float(safeHeight), 0.05f, 200.0f);
}

void RobotOpenGLWidget::paintGL()
{
    glClearColor(0.17f, 0.17f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!m_program.isLinked())
        return;

    m_program.bind();
    const QMatrix4x4 view = m_camera.viewMatrix();
    m_program.setUniformValue("projection", m_projection);
    m_program.setUniformValue("view", view);
    m_program.setUniformValue("cameraPosition", m_camera.position());
    m_program.setUniformValue("model", QMatrix4x4());
    m_program.setUniformValue("useLighting", false);
    m_program.setUniformValue("selected", false);
    drawGeometry(m_lineVao, m_lineVertexCount, GL_LINES);

    if (m_glbMeshes.empty()) {
        for (const QMatrix4x4 &linkTransform : m_linkTransforms)
            drawCube(linkTransform, view);
    } else {
        const QVector<GlbNode> &nodes = m_glbModel->nodes();
        for (int nodeIndex = 0; nodeIndex < nodes.size() && nodeIndex < m_glbNodeTransforms.size(); ++nodeIndex) {
            for (const int meshIndex : nodes[nodeIndex].meshIndices) {
                if (meshIndex < 0 || meshIndex >= int(m_glbMeshes.size())) continue;
                const GlbMesh &sourceMesh = m_glbModel->meshes()[meshIndex];
                m_program.setUniformValue("model", m_glbNodeTransforms[nodeIndex]);
                m_program.setUniformValue("useLighting", true);
                m_program.setUniformValue("baseColor", sourceMesh.baseColor);
                m_program.setUniformValue("metallic", sourceMesh.metallic);
                m_program.setUniformValue("roughness", sourceMesh.roughness);
                m_program.setUniformValue("selected", nodeIndex == m_selectedNodeIndex);
                GpuMesh &mesh = *m_glbMeshes[size_t(meshIndex)]; mesh.vao.bind();
                glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr); mesh.vao.release();
            }
        }
    }
    if (m_targetGizmo.isVisible()) {
        drawTargetGizmo();
    }
    m_program.release();
}

void RobotOpenGLWidget::setLinkTransforms(const QVector<QMatrix4x4> &linkTransforms)
{
    m_linkTransforms = linkTransforms;
    update();
}

void RobotOpenGLWidget::setGlbModel(const GlbModel &model)
{
    m_glbModel = &model;
    if (context()) { makeCurrent(); uploadGlbModel(); doneCurrent(); }
    update();
}

void RobotOpenGLWidget::setGlbNodeTransforms(const QVector<QMatrix4x4> &nodeTransforms)
{
    m_glbNodeTransforms = nodeTransforms;
    update();
}

void RobotOpenGLWidget::setIkTargetPosition(const QVector3D &position)
{
    m_targetGizmo.setPosition(position);
    update();
}

void RobotOpenGLWidget::clearGlbResources()
{
    for (const std::unique_ptr<GpuMesh> &mesh : m_glbMeshes) {
        mesh->vertices.destroy();
        mesh->indices.destroy();
        mesh->vao.destroy();
    }
    m_glbMeshes.clear();
}

void RobotOpenGLWidget::uploadGlbModel()
{
    clearGlbResources();
    if (!m_glbModel)
        return;
    for (const GlbMesh &source : m_glbModel->meshes()) {
        auto mesh = std::make_unique<GpuMesh>();
        mesh->vao.create(); mesh->vao.bind();
        mesh->vertices.create(); mesh->vertices.bind();
        mesh->vertices.allocate(source.vertices.constData(), source.vertices.size() * int(sizeof(float)));
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr); glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void *>(3 * sizeof(float))); glEnableVertexAttribArray(1);
        mesh->indices.create(); mesh->indices.bind();
        mesh->indices.allocate(source.indices.constData(), source.indices.size() * int(sizeof(quint32)));
        mesh->indexCount = source.indices.size();
        // EBO 绑定是 VAO 状态的一部分。此处若 release()，会在仍绑定该 VAO 时
        // 清空 GL_ELEMENT_ARRAY_BUFFER，随后 glDrawElements(nullptr) 会访问空地址。
        mesh->vertices.release();
        mesh->vao.release();
        m_glbMeshes.push_back(std::move(mesh));
    }
}

void RobotOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePosition = event->pos();
    if (event->button() == Qt::LeftButton) {
        m_leftPressPosition = event->pos();
        m_leftDragActive = false;
        m_draggingIkTarget = m_targetGizmo.beginDrag(event->pos(), size(), m_projection, m_camera.viewMatrix(), m_camera.position());
    }
    event->accept();
}

void RobotOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();

    if (event->buttons().testFlag(Qt::LeftButton)) {
        if (m_draggingIkTarget) {
            if (m_targetGizmo.drag(event->pos(), size(), m_projection, m_camera.viewMatrix())) {
                emit ikTargetMoved(m_targetGizmo.position());
                update();
            }
            event->accept();
            return;
        }
        m_leftDragActive |= (event->pos() - m_leftPressPosition).manhattanLength() > 3;
        m_camera.pan(float(delta.x()), float(delta.y()));
        update();
    } else if (event->buttons().testFlag(Qt::MiddleButton)) {
        m_camera.orbit(float(delta.x()), float(delta.y()));
        update();
    }
    event->accept();
}

void RobotOpenGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_draggingIkTarget) {
        m_targetGizmo.endDrag();
        m_draggingIkTarget = false;
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton && !m_leftDragActive && m_glbModel) {
        const int pickedNode = ScenePicker::pickNode(event->pos(), size(), m_projection, m_camera.viewMatrix(), *m_glbModel, m_glbNodeTransforms);
        m_selectedNodeIndex = pickedNode;
        if (pickedNode >= 0)
            m_targetGizmo.setPosition(m_glbNodeTransforms[pickedNode].map({0.0f, 0.0f, 0.0f}));
        emit componentSelected(pickedNode >= 0 ? m_glbModel->nodes()[pickedNode].name : QString(), pickedNode);
        update();
    }
    event->accept();
}

void RobotOpenGLWidget::wheelEvent(QWheelEvent *event)
{
    const float wheelSteps = float(event->angleDelta().y()) / 120.0f;
    m_camera.zoom(wheelSteps);
    update();
    event->accept();
}

void RobotOpenGLWidget::createSceneGeometry()
{
    std::vector<Vertex> lines;
    const auto addLine = [&lines](const QVector3D &a, const QVector3D &b, const QVector3D &color) {
        lines.push_back({a.x(), a.y(), a.z(), color.x(), color.y(), color.z()});
        lines.push_back({b.x(), b.y(), b.z(), color.x(), color.y(), color.z()});
    };

    constexpr int halfGrid = 10;
    for (int i = -halfGrid; i <= halfGrid; ++i) {
        const float coordinate = float(i);
        const QVector3D gridColor = i == 0 ? QVector3D(0.42f, 0.44f, 0.47f)
                                           : QVector3D(0.29f, 0.30f, 0.33f);
        addLine({coordinate, 0.0f, -halfGrid}, {coordinate, 0.0f, halfGrid}, gridColor);
        addLine({-halfGrid, 0.0f, coordinate}, {halfGrid, 0.0f, coordinate}, gridColor);
    }
    addLine({}, {1.5f, 0.0f, 0.0f}, {0.95f, 0.2f, 0.2f});
    addLine({}, {0.0f, 1.5f, 0.0f}, {0.2f, 0.95f, 0.25f});
    addLine({}, {0.0f, 0.0f, 1.5f}, {0.25f, 0.5f, 1.0f});

    m_lineVao.create();
    m_lineVao.bind();
    m_lineBuffer.create();
    m_lineBuffer.bind();
    m_lineBuffer.allocate(lines.data(), int(lines.size() * sizeof(Vertex)));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    m_lineBuffer.release();
    m_lineVao.release();
    m_lineVertexCount = int(lines.size());

    constexpr std::array<Vertex, 36> cube = {{
        {-0.5f,-0.5f, 0.5f, 0.95f,0.55f,0.15f}, { 0.5f,-0.5f, 0.5f, 0.95f,0.55f,0.15f}, { 0.5f, 0.5f, 0.5f, 0.95f,0.55f,0.15f},
        {-0.5f,-0.5f, 0.5f, 0.95f,0.55f,0.15f}, { 0.5f, 0.5f, 0.5f, 0.95f,0.55f,0.15f}, {-0.5f, 0.5f, 0.5f, 0.95f,0.55f,0.15f},
        { 0.5f,-0.5f,-0.5f, 0.80f,0.36f,0.08f}, {-0.5f,-0.5f,-0.5f, 0.80f,0.36f,0.08f}, {-0.5f, 0.5f,-0.5f, 0.80f,0.36f,0.08f},
        { 0.5f,-0.5f,-0.5f, 0.80f,0.36f,0.08f}, {-0.5f, 0.5f,-0.5f, 0.80f,0.36f,0.08f}, { 0.5f, 0.5f,-0.5f, 0.80f,0.36f,0.08f},
        {-0.5f,-0.5f,-0.5f, 0.72f,0.32f,0.08f}, {-0.5f,-0.5f, 0.5f, 0.72f,0.32f,0.08f}, {-0.5f, 0.5f, 0.5f, 0.72f,0.32f,0.08f},
        {-0.5f,-0.5f,-0.5f, 0.72f,0.32f,0.08f}, {-0.5f, 0.5f, 0.5f, 0.72f,0.32f,0.08f}, {-0.5f, 0.5f,-0.5f, 0.72f,0.32f,0.08f},
        { 0.5f,-0.5f, 0.5f, 1.00f,0.68f,0.20f}, { 0.5f,-0.5f,-0.5f, 1.00f,0.68f,0.20f}, { 0.5f, 0.5f,-0.5f, 1.00f,0.68f,0.20f},
        { 0.5f,-0.5f, 0.5f, 1.00f,0.68f,0.20f}, { 0.5f, 0.5f,-0.5f, 1.00f,0.68f,0.20f}, { 0.5f, 0.5f, 0.5f, 1.00f,0.68f,0.20f},
        {-0.5f, 0.5f, 0.5f, 1.00f,0.70f,0.24f}, { 0.5f, 0.5f, 0.5f, 1.00f,0.70f,0.24f}, { 0.5f, 0.5f,-0.5f, 1.00f,0.70f,0.24f},
        {-0.5f, 0.5f, 0.5f, 1.00f,0.70f,0.24f}, { 0.5f, 0.5f,-0.5f, 1.00f,0.70f,0.24f}, {-0.5f, 0.5f,-0.5f, 1.00f,0.70f,0.24f},
        {-0.5f,-0.5f,-0.5f, 0.60f,0.25f,0.05f}, { 0.5f,-0.5f,-0.5f, 0.60f,0.25f,0.05f}, { 0.5f,-0.5f, 0.5f, 0.60f,0.25f,0.05f},
        {-0.5f,-0.5f,-0.5f, 0.60f,0.25f,0.05f}, { 0.5f,-0.5f, 0.5f, 0.60f,0.25f,0.05f}, {-0.5f,-0.5f, 0.5f, 0.60f,0.25f,0.05f}
    }};
    m_cubeVao.create();
    m_cubeVao.bind();
    m_cubeBuffer.create();
    m_cubeBuffer.bind();
    m_cubeBuffer.allocate(cube.data(), int(cube.size() * sizeof(Vertex)));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    m_cubeBuffer.release();
    m_cubeVao.release();
    m_cubeVertexCount = int(cube.size());
}

void RobotOpenGLWidget::createGizmoResources()
{
    m_gizmoVao.create();
    m_gizmoVao.bind();
    m_gizmoBuffer.create();
    m_gizmoBuffer.bind();
    m_gizmoBuffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    m_gizmoBuffer.release();
    m_gizmoVao.release();
}

void RobotOpenGLWidget::drawTargetGizmo()
{
    const QVector3D center = m_targetGizmo.position();
    const float length = m_targetGizmo.axisLength(m_camera.position());
    std::vector<Vertex> vertices;
    const auto addLine = [&vertices](const QVector3D &start, const QVector3D &end, const QVector3D &color) {
        vertices.push_back({start.x(), start.y(), start.z(), color.x(), color.y(), color.z()});
        vertices.push_back({end.x(), end.y(), end.z(), color.x(), color.y(), color.z()});
    };
    addLine(center, center + QVector3D(length, 0.0f, 0.0f), {0.95f, 0.15f, 0.15f});
    addLine(center, center + QVector3D(0.0f, length, 0.0f), {0.15f, 0.95f, 0.20f});
    addLine(center, center + QVector3D(0.0f, 0.0f, length), {0.20f, 0.45f, 1.0f});
    const float cap = length * 0.10f;
    addLine(center + QVector3D(length, 0, 0), center + QVector3D(length - cap, cap, 0), {0.95f, 0.15f, 0.15f});
    addLine(center + QVector3D(length, 0, 0), center + QVector3D(length - cap, -cap, 0), {0.95f, 0.15f, 0.15f});
    addLine(center + QVector3D(0, length, 0), center + QVector3D(cap, length - cap, 0), {0.15f, 0.95f, 0.20f});
    addLine(center + QVector3D(0, length, 0), center + QVector3D(-cap, length - cap, 0), {0.15f, 0.95f, 0.20f});
    addLine(center + QVector3D(0, 0, length), center + QVector3D(cap, 0, length - cap), {0.20f, 0.45f, 1.0f});
    addLine(center + QVector3D(0, 0, length), center + QVector3D(-cap, 0, length - cap), {0.20f, 0.45f, 1.0f});
    m_program.setUniformValue("model", QMatrix4x4());
    m_program.setUniformValue("useLighting", false);
    m_program.setUniformValue("selected", false);
    glDisable(GL_DEPTH_TEST);
    m_gizmoVao.bind();
    m_gizmoBuffer.bind();
    m_gizmoBuffer.allocate(vertices.data(), int(vertices.size() * sizeof(Vertex)));
    m_gizmoBuffer.release();
    glDrawArrays(GL_LINES, 0, int(vertices.size()));
    m_gizmoVao.release();
    glEnable(GL_DEPTH_TEST);
}

void RobotOpenGLWidget::drawGeometry(QOpenGLVertexArrayObject &vao, int count, unsigned int primitive)
{
    if (count == 0)
        return;
    vao.bind();
    glDrawArrays(primitive, 0, count);
    vao.release();
}

void RobotOpenGLWidget::drawCube(const QMatrix4x4 &model, const QMatrix4x4 &view)
{
    Q_UNUSED(view);
    m_program.setUniformValue("model", model);
    m_program.setUniformValue("useLighting", false);
    m_program.setUniformValue("selected", false);
    drawGeometry(m_cubeVao, m_cubeVertexCount, GL_TRIANGLES);
}

void RobotOpenGLWidget::resetCamera()
{
    m_camera.reset();
    update();
}
