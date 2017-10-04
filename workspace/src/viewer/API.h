#pragma once

#include <string>
#include <QObject>

class API : public QObject
{
    Q_OBJECT

public:
    API();
    virtual ~API();

signals:

public slots:
    int sum(int a, int b);

private:

};
