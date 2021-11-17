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

    void setAxes(double xStart, double xStop, double xMajorDivision, double yStart, double yStop, double yMajorDivision, int xLabelEvery = 0, int yLabelEvery = 0);
    QGraphicsScene *getScene();
    QGraphicsLineItem *createSegment(double x1, double y1, double x2, double y2, QPen pen);
    QGraphicsTextItem *createLabel(double x, double y, double value);
    void clear();

private:
    QGraphicsScene *scene;
    double xScale;
    double yScale;
    double xStart;
    double yStart;
    double xStop;
    double yStop;
};

#endif // PLOT_H
