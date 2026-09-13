cmake_minimum_required(VERSION 3.19)
project(robot LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

qt_standard_project_setup()

qt_add_executable(robot
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    render/robotopenglwidget.cpp
    render/robotopenglwidget.h
    render/orbitcamera.cpp
    render/orbitcamera.h
    render/scenepicker.cpp
    render/scenepicker.h
    render/targetgizmo.cpp
    render/targetgizmo.h
    robot/glbmodel.cpp
    robot/glbmodel.h
    robot/tinygltfloader.cpp
    robot/tinygltfloader.h
    tinygltf/tiny_gltf_v3.cpp
    robot/joint.cpp
    robot/joint.h
    robot/jointanimator.cpp
    robot/jointanimator.h
    robot/iksolver.cpp
    robot/iksolver.h
    robot/kinematicstate.h
    robot/trajectoryplayer.cpp
    robot/trajectoryplayer.h
    robot/robotmodel.cpp
    robot/robotmodel.h
)

target_link_libraries(robot
    PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::OpenGL
        Qt::OpenGLWidgets
)

# 让从构建目录直接启动的程序也能找到用户放在项目根目录的模型。
add_custom_command(TARGET robot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/irb4600.glb
        $<TARGET_FILE_DIR:robot>/irb4600.glb
)

install(TARGETS robot
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET robot
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
