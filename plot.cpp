#include "plot.h"

Plot::Plot()
{
    scene = new QGraphicsScene();
}

void Plot::setAxes(double xMax, double xMajorDivision, double yMax, double yMajorDivision)
{
    xScale = PLOT_WIDTH / xMax;
    yScale = PLOT_HEIGHT / yMax;

    scene->clear();

    double x = 0.0;
    bool doLabel = true;
    while (x <= xMax) {
        scene->addLine(x * xScale, 0, x * xScale, PLOT_HEIGHT);

        if (doLabel) { // Label every other X scale point
            QGraphicsTextItem *text;
            char labelText[16];
            sprintf(labelText, "%d", (int) (x + 0.5));
            text = scene->addText(labelText);
            double offset = 6.0 * strlen(labelText);
            text->setPos(x * xScale - offset, PLOT_HEIGHT + 10);
        }

        doLabel = !doLabel;
        x += xMajorDivision;
    }

    double y = 0;
    while (y <= yMax) {
        scene->addLine(0, PLOT_HEIGHT - y * yScale, PLOT_WIDTH, PLOT_HEIGHT - y * yScale);

        QGraphicsTextItem *text;
        char labelText[16];
        sprintf(labelText, "%d", (int) (y + 0.5));
        text = scene->addText(labelText);
        double offset = 12.0 * strlen(labelText);
        text->setPos(-10 - offset, PLOT_HEIGHT - y * yScale - 10);

        y += yMajorDivision;
    }
}

QGraphicsScene *Plot::getScene()
{
    return scene;
}

QGraphicsLineItem *Plot::createSegment(double x1, double y1, double x2, double y2, QPen pen)
{
    return scene->addLine(x1 * xScale, PLOT_HEIGHT - y1 * yScale, x2 * xScale, PLOT_HEIGHT - y2 * yScale, pen);
}

QGraphicsTextItem *Plot::createLabel(double x, double y, double value)
{
    QGraphicsTextItem *text;
    char labelText[16];
    sprintf(labelText, "%.1fv", value + 0.04);
    text = scene->addText(labelText);
    text->setPos(x * xScale + 5, PLOT_HEIGHT - y * yScale - 10);

    return text;
}
