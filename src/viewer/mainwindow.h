#pragma once

#include <QTimer>
#include <QWebInspector>
#include <QMainWindow>
#include "API.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(API *api, QWidget *parent = 0);
    ~MainWindow();

public slots:
    void on_actionInspector_triggered();
    void on_actionAccessibility_triggered();
    void on_actionSuperBigleux_triggered();

 private:
    bool accessibility;
    bool superBigleuxMode;
    QAction *inspectorAction;
    QAction *accessibilityAction;
    QAction *superBigleuxAction;
    QWebInspector *inspector;
    Ui::MainWindow *ui;
    API *api;
};
