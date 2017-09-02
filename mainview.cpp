#include "MainView.h"
#include "investigation.h"

#include <QDebug>
#include <QPainter>
#include <QMessageBox>
#include <QCloseEvent>


MainView::MainView(QWidget *parent)
    : QWidget(parent)
{
    investigation = new Investigation(this);
    
    for (int i = 0; i < (int)investigation->getThreadsCount(); ++i)
    {
        auto view = new MainView(investigation, i);
        view->move(i * QPoint(100, 100));
        view->show();
    }

    connect(investigation, &Investigation::modified, this, &MainView::onModified);

    investigation->genetic();
}

MainView::MainView(Investigation* in, int index, QWidget *parent) : QWidget(parent)
{
    investigation = in;
    viewIndex = index;

    connect(in, &Investigation::modified, this, &MainView::onModified);

    setWindowFlags(Qt::SubWindow);
    setWindowOpacity(0.7);
}

void MainView::paintEvent(QPaintEvent* e)
{
    if (!investigation)
    {
        return QWidget::paintEvent(e);
    }

    QPainter painter(this);
    painter.setPen(Qt::black);

  //  drawPolygon(painter, width(), height(), investigation->getOptimalPolygon());


    auto points = investigation->getPoints(viewIndex, width(), height());

    painter.drawPoints(points.data(), points.size());
}

void MainView::onModified(int ind)
{
    if (ind == viewIndex)
    {
        QString title;
        if (ind != -1)
        {
            title = QString::number(viewIndex) + ": ";
        }

        title += QString::number(investigation->getSquare(ind), 'g', 12);

        setWindowTitle(title);
        repaint();
    }
}

void MainView::closeEvent(QCloseEvent *event)
{
    if (QMessageBox::question(this, "Close Sofa", "Do you really want to stop investigation") == QMessageBox::Yes)
    {
        QWidget::closeEvent(event);
    }
    else
    {
        event->ignore();
    }
}

