#include "plot.h"

Plot::Plot()
{
    scene = new QGraphicsScene();
}

void Plot::setAxes(double _xStart, double _xStop, double xMajorDivision, double _yStart, double _yStop, double yMajorDivision, int xLabelEvery, int yLabelEvery)
{
    xStart = _xStart;
    xStop = _xStop;
    yStart = _yStart;
    yStop = _yStop;

    xScale = PLOT_WIDTH / (xStop - xStart);
    yScale = PLOT_HEIGHT /(yStop - yStart);

    double rounding = 0.5;

    scene->clear();

    if (xScale < 0) {
        if (xMajorDivision > 0) {
            xMajorDivision = -xMajorDivision;
        }
    } else {
        if (xMajorDivision < 0) {
            xMajorDivision = -xMajorDivision;
        }
    }
    double x = xStart;
    int i = 0;
    while (x <= xStop) {
        scene->addLine((x - xStart) * xScale, 0, (x - xStart) * xScale, PLOT_HEIGHT);
        rounding = (x > 0) ? 0.5 : -0.5;
        if (xLabelEvery == 0 || (i % xLabelEvery) == 0) {
            QGraphicsTextItem *text;
            char labelText[16];
            sprintf(labelText, "%d", (int) (x + rounding));
            text = scene->addText(labelText);
            double offset = 6.0 * strlen(labelText);
            text->setPos((x - xStart) * xScale - offset, PLOT_HEIGHT + 10);
        }

        x += xMajorDivision;
        i++;
    }

    if (yScale < 0) {
        if (yMajorDivision > 0) {
            yMajorDivision = -yMajorDivision;
        }
    } else {
        if (yMajorDivision < 0) {
            yMajorDivision = -yMajorDivision;
        }
    }
    double y = yStart;
    i = 0;
    while (y <= yStop) {
        scene->addLine(0, PLOT_HEIGHT - (y - yStart) * yScale, PLOT_WIDTH, PLOT_HEIGHT - (y - yStart) * yScale);
        rounding = (y > 0) ? 0.5 : -0.5;
        if (yLabelEvery == 0 || (i % yLabelEvery) == 0) {
            QGraphicsTextItem *text;
            char labelText[16];
            sprintf(labelText, "%d", (int) (y + rounding));
            text = scene->addText(labelText);
            double offset = 12.0 * strlen(labelText);
            text->setPos(-10 - offset, PLOT_HEIGHT - (y - yStart) * yScale - 10);
        }

        y += yMajorDivision;
        i++;
    }
}

QGraphicsScene *Plot::getScene()
{
    return scene;
}

QGraphicsLineItem *Plot::createSegment(double x1, double y1, double x2, double y2, QPen pen)
{
    double x1_ = (x1 - xStart) * xScale;
    double y1_ = PLOT_HEIGHT - (y1 - yStart) * yScale;
    double x2_ = (x2 - xStart) * xScale;
    double y2_ = PLOT_HEIGHT - (y2 - yStart) * yScale;

    if (x1_ < 0.0 || x1_ > PLOT_WIDTH || x2_ < 0.0 || x2_ > PLOT_WIDTH || y1_ < 0.0 || y1_ > PLOT_HEIGHT || y2_ < 0.0 || y2_ > PLOT_HEIGHT) {
        return scene->addLine(0.0, PLOT_HEIGHT, 0.0, PLOT_HEIGHT, pen); // Return an invisible segment if out of bounds
    }

    return scene->addLine(x1_, y1_, x2_, y2_, pen);
}

QGraphicsTextItem *Plot::createLabel(double x, double y, double value)
{
    QGraphicsTextItem *text;
    char labelText[16];
    sprintf(labelText, "%.1fv", value + 0.04);
    text = scene->addText(labelText);
    text->setPos((x - xStart) * xScale + 5, PLOT_HEIGHT - (y - yStart) * yScale - 10);

    return text;
}

void Plot::clear()
{
    scene->clear();
}
