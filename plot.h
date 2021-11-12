#ifndef PLOT_H
#define PLOT_H

#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>

#define PLOT_WIDTH 430
#define PLOT_HEIGHT 370

class Plot
{
public:
    Plot();

    void setAxes(double xMax, double xMajorDivision, double yMax, double yMajorDivision);
    QGraphicsScene *getScene();
    QGraphicsLineItem *createSegment(double x1, double y1, double x2, double y2, QPen pen);
    QGraphicsTextItem *createLabel(double x, double y, double value);

private:
    QGraphicsScene *scene;
    double xScale;
    double yScale;
};

#endif // PLOT_H
