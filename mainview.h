#pragma once

#include <QWidget>
#include <QPointer>

class Investigation;


class MainView : public QWidget
{
    Q_OBJECT

public:
    MainView(QWidget *parent = nullptr);

protected:
    MainView(Investigation* investigation, int ind, QWidget *parent = nullptr);

protected:

    void paintEvent(QPaintEvent *event) override;

    void closeEvent(QCloseEvent *event) override;
    
private:
    
    void onModified(int ind);

private:

    int viewIndex = -1;
    QPointer<Investigation> investigation;
};

