cmake_minimum_required(VERSION 2.8.11)
project(viewer)

# Find the QtWidgets library
find_package(Qt5Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# QT resource file
qt5_add_resources(RESOURCES www.qrc)

# Adding mainwindow QT form
qt5_wrap_ui(ui_mainwindow.h mainwindow.ui)

# Sources
set (SOURCES
    main.cpp
    mainwindow.cpp
    API.cpp
    ui_mainwindow.h
    ${RESOURCES}
)

# Tell CMake to create the helloworld executable
add_executable(viewer ${SOURCES})

# Widgets
qt5_use_modules(viewer
    Core
    Gui
    Network
    WebKit
    WebKitWidgets
    Widgets
)
