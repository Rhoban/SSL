#pragma once

#include <string>
#include <QObject>

class API : public QObject
{
    Q_OBJECT

public:
    API(bool simulation, bool yellow);
    virtual ~API();

    bool simulation;
    bool yellow;

signals:

public slots:
    bool isSimulation();
    bool isYellow();

private:

};
