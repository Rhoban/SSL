#include <QWebFrame>
#include <QWebInspector>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(API *api, QWidget *parent) :
    QMainWindow(parent),
    accessibility(false),
    superBigleuxMode(false),
    inspector(NULL),
    ui(new Ui::MainWindow),
    api(api)
{
  ui->setupUi(this);

  ui->webView->page()->mainFrame()->addToJavaScriptWindowObject("api", api);

  ui->horizontalLayout->setMargin(0);
  ui->horizontalLayout->setSpacing(0);
  ui->webView->settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);

  inspectorAction = new QAction(tr("Show inspector"), this);
  inspectorAction->setShortcut(QKeySequence::Find);
  connect(inspectorAction, SIGNAL(triggered()), this, SLOT(on_actionInspector_triggered()));
  addAction(inspectorAction);

  accessibilityAction = new QAction(tr("Accessiblity"), this);
  accessibilityAction->setShortcut(QKeySequence::Bold);
  connect(accessibilityAction, SIGNAL(triggered()), this, SLOT(on_actionAccessibility_triggered()));
  addAction(accessibilityAction);

  superBigleuxAction = new QAction(tr("SuperBigleux"), this);
  superBigleuxAction->setShortcut(QKeySequence::Save);
  connect(superBigleuxAction, SIGNAL(triggered()), this, SLOT(on_actionSuperBigleux_triggered()));
  addAction(superBigleuxAction);

  showMaximized();
}

MainWindow::~MainWindow()
{
  if (inspector) {
    delete inspector;
  }
  delete accessibilityAction;
  delete superBigleuxAction;
  delete inspectorAction;
  delete ui;
}

void MainWindow::on_actionInspector_triggered()
{
  if (inspector == NULL) {
    inspector = new QWebInspector;
    inspector->setPage(ui->webView->page());
  }
  inspector->setVisible(true);
}

void MainWindow::on_actionAccessibility_triggered()
{
  accessibility = !accessibility;

  if (accessibility) {
    ui->webView->setZoomFactor(1.5);
    setGeometry(x(), y(), 1.5*width(), 1.5*height());
  } else {
    ui->webView->setZoomFactor(1);
    setGeometry(x(), y(), width()/1.5, height()/1.5);
  }
}

void MainWindow::on_actionSuperBigleux_triggered()
{
  superBigleuxMode = !superBigleuxMode;

  if (superBigleuxMode) {
    ui->webView->setZoomFactor(2.5);
    setGeometry(x(), y(), 2.5*width(), 2.5*height());
  } else {
    ui->webView->setZoomFactor(1);
    setGeometry(x(), y(), width()/2.5, height()/2.5);
  }
}
