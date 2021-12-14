/********************************************************************************
** Form generated from reading UI file 'valveanalyser.ui'
**
** Created by: Qt User Interface Compiler version 6.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VALVEANALYSER_H
#define UI_VALVEANALYSER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ValveAnalyser
{
public:
    QAction *actionPrint;
    QAction *actionQuit;
    QAction *actionOptions;
    QAction *actionContents;
    QAction *actionAbout;
    QWidget *centralwidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_22;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_7;
    QLineEdit *deviceName;
    QHBoxLayout *horizontalLayout_14;
    QLabel *deviceTypeLabel;
    QComboBox *deviceType;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_4;
    QComboBox *testType;
    QSpacerItem *verticalSpacer_4;
    QHBoxLayout *horizontalLayout_3;
    QLabel *heaterLabel;
    QLineEdit *heaterVoltage;
    QSpacerItem *horizontalSpacer_7;
    QHBoxLayout *horizontalLayout_16;
    QSpacerItem *horizontalSpacer_15;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_4;
    QLabel *anodeLabel;
    QLineEdit *anodeStart;
    QLineEdit *anodeStop;
    QLineEdit *anodeStep;
    QHBoxLayout *gridGroup;
    QLabel *gridLabel;
    QLineEdit *gridStart;
    QLineEdit *gridStop;
    QLineEdit *gridStep;
    QHBoxLayout *screenGroup;
    QLabel *screenLabel;
    QLineEdit *screenStart;
    QLineEdit *screenStop;
    QLineEdit *screenStep;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_5;
    QLineEdit *iaMax;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_6;
    QLineEdit *pMax;
    QSpacerItem *horizontalSpacer_3;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *heaterLayout;
    QPushButton *heaterButton;
    QSpacerItem *horizontalSpacer_8;
    QHBoxLayout *horizontalLayout_11;
    QLabel *heaterVLabel;
    QSpacerItem *horizontalSpacer_9;
    QLCDNumber *heaterVlcd;
    QHBoxLayout *horizontalLayout_12;
    QLabel *heaterILabel;
    QSpacerItem *horizontalSpacer_10;
    QLCDNumber *heaterIlcd;
    QSpacerItem *verticalSpacer_3;
    QHBoxLayout *horizontalLayout_15;
    QPushButton *runButton;
    QSpacerItem *horizontalSpacer_12;
    QProgressBar *progressBar;
    QSpacerItem *verticalSpacer;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_2;
    QLabel *plotTitle;
    QGraphicsView *graphicsView;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_6;
    QCheckBox *showMeasuredValues;
    QSpacerItem *horizontalSpacer_13;
    QCheckBox *showModelledValues;
    QSpacerItem *horizontalSpacer_11;
    QSpacerItem *verticalSpacer_6;
    QSpacerItem *horizontalSpacer_4;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_8;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_10;
    QComboBox *modelSelection;
    QPushButton *fitModelButton;
    QHBoxLayout *horizontalLayout_8;
    QLabel *par1Label;
    QLineEdit *par1Value;
    QHBoxLayout *horizontalLayout_10;
    QLabel *par2Label;
    QLineEdit *par2Value;
    QHBoxLayout *horizontalLayout_13;
    QLabel *par3Label;
    QLineEdit *par3Value;
    QHBoxLayout *horizontalLayout_18;
    QLabel *par4Label;
    QLineEdit *par4Value;
    QHBoxLayout *horizontalLayout_19;
    QLabel *par5Label;
    QLineEdit *par5Value;
    QHBoxLayout *horizontalLayout_20;
    QLabel *par6Label;
    QLineEdit *par6Value;
    QHBoxLayout *horizontalLayout_21;
    QLabel *par7Label;
    QLineEdit *par7Value;
    QSpacerItem *verticalSpacer_5;
    QSpacerItem *horizontalSpacer_5;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ValveAnalyser)
    {
        if (ValveAnalyser->objectName().isEmpty())
            ValveAnalyser->setObjectName(QString::fromUtf8("ValveAnalyser"));
        ValveAnalyser->resize(1119, 657);
        actionPrint = new QAction(ValveAnalyser);
        actionPrint->setObjectName(QString::fromUtf8("actionPrint"));
        actionQuit = new QAction(ValveAnalyser);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionOptions = new QAction(ValveAnalyser);
        actionOptions->setObjectName(QString::fromUtf8("actionOptions"));
        actionContents = new QAction(ValveAnalyser);
        actionContents->setObjectName(QString::fromUtf8("actionContents"));
        actionAbout = new QAction(ValveAnalyser);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        centralwidget = new QWidget(ValveAnalyser);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(19, 19, 1081, 592));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_22 = new QHBoxLayout();
        horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
        pushButton_2 = new QPushButton(horizontalLayoutWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_22->addWidget(pushButton_2);

        pushButton = new QPushButton(horizontalLayoutWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_22->addWidget(pushButton);


        verticalLayout->addLayout(horizontalLayout_22);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_7 = new QLabel(horizontalLayoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_6->addWidget(label_7);

        deviceName = new QLineEdit(horizontalLayoutWidget);
        deviceName->setObjectName(QString::fromUtf8("deviceName"));
        deviceName->setMinimumSize(QSize(160, 0));
        deviceName->setMaximumSize(QSize(160, 16777215));

        horizontalLayout_6->addWidget(deviceName);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        deviceTypeLabel = new QLabel(horizontalLayoutWidget);
        deviceTypeLabel->setObjectName(QString::fromUtf8("deviceTypeLabel"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(deviceTypeLabel->sizePolicy().hasHeightForWidth());
        deviceTypeLabel->setSizePolicy(sizePolicy);
        deviceTypeLabel->setMinimumSize(QSize(100, 0));

        horizontalLayout_14->addWidget(deviceTypeLabel);

        deviceType = new QComboBox(horizontalLayoutWidget);
        deviceType->setObjectName(QString::fromUtf8("deviceType"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(deviceType->sizePolicy().hasHeightForWidth());
        deviceType->setSizePolicy(sizePolicy1);
        deviceType->setMinimumSize(QSize(160, 0));
        deviceType->setMaximumSize(QSize(160, 16777215));

        horizontalLayout_14->addWidget(deviceType);


        verticalLayout->addLayout(horizontalLayout_14);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        label_4 = new QLabel(horizontalLayoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_17->addWidget(label_4);

        testType = new QComboBox(horizontalLayoutWidget);
        testType->setObjectName(QString::fromUtf8("testType"));
        sizePolicy1.setHeightForWidth(testType->sizePolicy().hasHeightForWidth());
        testType->setSizePolicy(sizePolicy1);
        testType->setMinimumSize(QSize(160, 0));
        testType->setMaximumSize(QSize(160, 16777215));

        horizontalLayout_17->addWidget(testType);


        verticalLayout->addLayout(horizontalLayout_17);

        verticalSpacer_4 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        heaterLabel = new QLabel(horizontalLayoutWidget);
        heaterLabel->setObjectName(QString::fromUtf8("heaterLabel"));
        sizePolicy.setHeightForWidth(heaterLabel->sizePolicy().hasHeightForWidth());
        heaterLabel->setSizePolicy(sizePolicy);
        heaterLabel->setMinimumSize(QSize(100, 0));

        horizontalLayout_3->addWidget(heaterLabel);

        heaterVoltage = new QLineEdit(horizontalLayoutWidget);
        heaterVoltage->setObjectName(QString::fromUtf8("heaterVoltage"));
        sizePolicy1.setHeightForWidth(heaterVoltage->sizePolicy().hasHeightForWidth());
        heaterVoltage->setSizePolicy(sizePolicy1);
        heaterVoltage->setMinimumSize(QSize(50, 0));
        heaterVoltage->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_3->addWidget(heaterVoltage);

        horizontalSpacer_7 = new QSpacerItem(112, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_7);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        horizontalSpacer_15 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_15);

        label = new QLabel(horizontalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setMinimumSize(QSize(50, 20));
        label->setMaximumSize(QSize(50, 20));
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout_16->addWidget(label);

        label_2 = new QLabel(horizontalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy2);
        label_2->setMinimumSize(QSize(50, 0));
        label_2->setMaximumSize(QSize(50, 16777215));
        label_2->setAlignment(Qt::AlignCenter);

        horizontalLayout_16->addWidget(label_2);

        label_3 = new QLabel(horizontalLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        sizePolicy2.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy2);
        label_3->setMinimumSize(QSize(50, 0));
        label_3->setMaximumSize(QSize(50, 16777215));
        label_3->setAlignment(Qt::AlignCenter);

        horizontalLayout_16->addWidget(label_3);


        verticalLayout->addLayout(horizontalLayout_16);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        anodeLabel = new QLabel(horizontalLayoutWidget);
        anodeLabel->setObjectName(QString::fromUtf8("anodeLabel"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(100);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(anodeLabel->sizePolicy().hasHeightForWidth());
        anodeLabel->setSizePolicy(sizePolicy3);
        anodeLabel->setMinimumSize(QSize(100, 0));

        horizontalLayout_4->addWidget(anodeLabel);

        anodeStart = new QLineEdit(horizontalLayoutWidget);
        anodeStart->setObjectName(QString::fromUtf8("anodeStart"));
        sizePolicy1.setHeightForWidth(anodeStart->sizePolicy().hasHeightForWidth());
        anodeStart->setSizePolicy(sizePolicy1);
        anodeStart->setMinimumSize(QSize(50, 0));
        anodeStart->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_4->addWidget(anodeStart);

        anodeStop = new QLineEdit(horizontalLayoutWidget);
        anodeStop->setObjectName(QString::fromUtf8("anodeStop"));
        sizePolicy1.setHeightForWidth(anodeStop->sizePolicy().hasHeightForWidth());
        anodeStop->setSizePolicy(sizePolicy1);
        anodeStop->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_4->addWidget(anodeStop);

        anodeStep = new QLineEdit(horizontalLayoutWidget);
        anodeStep->setObjectName(QString::fromUtf8("anodeStep"));
        sizePolicy1.setHeightForWidth(anodeStep->sizePolicy().hasHeightForWidth());
        anodeStep->setSizePolicy(sizePolicy1);
        anodeStep->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_4->addWidget(anodeStep);


        verticalLayout->addLayout(horizontalLayout_4);

        gridGroup = new QHBoxLayout();
        gridGroup->setObjectName(QString::fromUtf8("gridGroup"));
        gridLabel = new QLabel(horizontalLayoutWidget);
        gridLabel->setObjectName(QString::fromUtf8("gridLabel"));
        sizePolicy.setHeightForWidth(gridLabel->sizePolicy().hasHeightForWidth());
        gridLabel->setSizePolicy(sizePolicy);
        gridLabel->setMinimumSize(QSize(100, 0));

        gridGroup->addWidget(gridLabel);

        gridStart = new QLineEdit(horizontalLayoutWidget);
        gridStart->setObjectName(QString::fromUtf8("gridStart"));
        sizePolicy1.setHeightForWidth(gridStart->sizePolicy().hasHeightForWidth());
        gridStart->setSizePolicy(sizePolicy1);
        gridStart->setMinimumSize(QSize(50, 0));
        gridStart->setMaximumSize(QSize(50, 16777215));

        gridGroup->addWidget(gridStart);

        gridStop = new QLineEdit(horizontalLayoutWidget);
        gridStop->setObjectName(QString::fromUtf8("gridStop"));
        sizePolicy1.setHeightForWidth(gridStop->sizePolicy().hasHeightForWidth());
        gridStop->setSizePolicy(sizePolicy1);
        gridStop->setMinimumSize(QSize(50, 0));
        gridStop->setMaximumSize(QSize(50, 16777215));

        gridGroup->addWidget(gridStop);

        gridStep = new QLineEdit(horizontalLayoutWidget);
        gridStep->setObjectName(QString::fromUtf8("gridStep"));
        sizePolicy1.setHeightForWidth(gridStep->sizePolicy().hasHeightForWidth());
        gridStep->setSizePolicy(sizePolicy1);
        gridStep->setMinimumSize(QSize(50, 0));
        gridStep->setMaximumSize(QSize(50, 16777215));

        gridGroup->addWidget(gridStep);


        verticalLayout->addLayout(gridGroup);

        screenGroup = new QHBoxLayout();
        screenGroup->setObjectName(QString::fromUtf8("screenGroup"));
        screenLabel = new QLabel(horizontalLayoutWidget);
        screenLabel->setObjectName(QString::fromUtf8("screenLabel"));
        sizePolicy.setHeightForWidth(screenLabel->sizePolicy().hasHeightForWidth());
        screenLabel->setSizePolicy(sizePolicy);
        screenLabel->setMinimumSize(QSize(100, 0));

        screenGroup->addWidget(screenLabel);

        screenStart = new QLineEdit(horizontalLayoutWidget);
        screenStart->setObjectName(QString::fromUtf8("screenStart"));
        sizePolicy1.setHeightForWidth(screenStart->sizePolicy().hasHeightForWidth());
        screenStart->setSizePolicy(sizePolicy1);
        screenStart->setMinimumSize(QSize(50, 0));
        screenStart->setMaximumSize(QSize(50, 16777215));

        screenGroup->addWidget(screenStart);

        screenStop = new QLineEdit(horizontalLayoutWidget);
        screenStop->setObjectName(QString::fromUtf8("screenStop"));
        sizePolicy1.setHeightForWidth(screenStop->sizePolicy().hasHeightForWidth());
        screenStop->setSizePolicy(sizePolicy1);
        screenStop->setMinimumSize(QSize(50, 0));
        screenStop->setMaximumSize(QSize(50, 16777215));

        screenGroup->addWidget(screenStop);

        screenStep = new QLineEdit(horizontalLayoutWidget);
        screenStep->setObjectName(QString::fromUtf8("screenStep"));
        sizePolicy1.setHeightForWidth(screenStep->sizePolicy().hasHeightForWidth());
        screenStep->setSizePolicy(sizePolicy1);
        screenStep->setMinimumSize(QSize(50, 0));
        screenStep->setMaximumSize(QSize(50, 16777215));

        screenGroup->addWidget(screenStep);


        verticalLayout->addLayout(screenGroup);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_5 = new QLabel(horizontalLayoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setMinimumSize(QSize(120, 0));
        label_5->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_2->addWidget(label_5);

        iaMax = new QLineEdit(horizontalLayoutWidget);
        iaMax->setObjectName(QString::fromUtf8("iaMax"));
        sizePolicy1.setHeightForWidth(iaMax->sizePolicy().hasHeightForWidth());
        iaMax->setSizePolicy(sizePolicy1);
        iaMax->setMinimumSize(QSize(50, 0));
        iaMax->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_2->addWidget(iaMax);

        horizontalSpacer_2 = new QSpacerItem(112, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_6 = new QLabel(horizontalLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setMinimumSize(QSize(120, 0));
        label_6->setMaximumSize(QSize(16777215, 16777215));

        horizontalLayout_5->addWidget(label_6);

        pMax = new QLineEdit(horizontalLayoutWidget);
        pMax->setObjectName(QString::fromUtf8("pMax"));
        sizePolicy1.setHeightForWidth(pMax->sizePolicy().hasHeightForWidth());
        pMax->setSizePolicy(sizePolicy1);
        pMax->setMinimumSize(QSize(50, 0));
        pMax->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_5->addWidget(pMax);

        horizontalSpacer_3 = new QSpacerItem(112, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_3);


        verticalLayout_4->addLayout(horizontalLayout_5);


        verticalLayout->addLayout(verticalLayout_4);

        verticalSpacer_2 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_2);

        heaterLayout = new QHBoxLayout();
        heaterLayout->setObjectName(QString::fromUtf8("heaterLayout"));
        heaterButton = new QPushButton(horizontalLayoutWidget);
        heaterButton->setObjectName(QString::fromUtf8("heaterButton"));
        sizePolicy1.setHeightForWidth(heaterButton->sizePolicy().hasHeightForWidth());
        heaterButton->setSizePolicy(sizePolicy1);
        heaterButton->setMinimumSize(QSize(80, 0));
        heaterButton->setCheckable(false);

        heaterLayout->addWidget(heaterButton);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        heaterLayout->addItem(horizontalSpacer_8);


        verticalLayout->addLayout(heaterLayout);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        heaterVLabel = new QLabel(horizontalLayoutWidget);
        heaterVLabel->setObjectName(QString::fromUtf8("heaterVLabel"));
        sizePolicy2.setHeightForWidth(heaterVLabel->sizePolicy().hasHeightForWidth());
        heaterVLabel->setSizePolicy(sizePolicy2);
        heaterVLabel->setMinimumSize(QSize(100, 0));

        horizontalLayout_11->addWidget(heaterVLabel);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_9);

        heaterVlcd = new QLCDNumber(horizontalLayoutWidget);
        heaterVlcd->setObjectName(QString::fromUtf8("heaterVlcd"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Minimum);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(heaterVlcd->sizePolicy().hasHeightForWidth());
        heaterVlcd->setSizePolicy(sizePolicy4);
        heaterVlcd->setMinimumSize(QSize(133, 0));
        heaterVlcd->setDigitCount(6);
        heaterVlcd->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_11->addWidget(heaterVlcd);


        verticalLayout->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        heaterILabel = new QLabel(horizontalLayoutWidget);
        heaterILabel->setObjectName(QString::fromUtf8("heaterILabel"));
        sizePolicy2.setHeightForWidth(heaterILabel->sizePolicy().hasHeightForWidth());
        heaterILabel->setSizePolicy(sizePolicy2);
        heaterILabel->setMinimumSize(QSize(100, 0));

        horizontalLayout_12->addWidget(heaterILabel);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_12->addItem(horizontalSpacer_10);

        heaterIlcd = new QLCDNumber(horizontalLayoutWidget);
        heaterIlcd->setObjectName(QString::fromUtf8("heaterIlcd"));
        sizePolicy4.setHeightForWidth(heaterIlcd->sizePolicy().hasHeightForWidth());
        heaterIlcd->setSizePolicy(sizePolicy4);
        heaterIlcd->setMinimumSize(QSize(133, 0));
        heaterIlcd->setLineWidth(1);
        heaterIlcd->setDigitCount(6);
        heaterIlcd->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_12->addWidget(heaterIlcd);


        verticalLayout->addLayout(horizontalLayout_12);

        verticalSpacer_3 = new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout->addItem(verticalSpacer_3);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        runButton = new QPushButton(horizontalLayoutWidget);
        runButton->setObjectName(QString::fromUtf8("runButton"));
        runButton->setCheckable(true);

        horizontalLayout_15->addWidget(runButton);

        horizontalSpacer_12 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_15->addItem(horizontalSpacer_12);


        verticalLayout->addLayout(horizontalLayout_15);

        progressBar = new QProgressBar(horizontalLayoutWidget);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setValue(24);
        progressBar->setTextVisible(false);

        verticalLayout->addWidget(progressBar);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);

        horizontalSpacer = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        plotTitle = new QLabel(horizontalLayoutWidget);
        plotTitle->setObjectName(QString::fromUtf8("plotTitle"));
        QSizePolicy sizePolicy5(QSizePolicy::Ignored, QSizePolicy::Preferred);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(plotTitle->sizePolicy().hasHeightForWidth());
        plotTitle->setSizePolicy(sizePolicy5);

        verticalLayout_2->addWidget(plotTitle);

        graphicsView = new QGraphicsView(horizontalLayoutWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        QSizePolicy sizePolicy6(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy6);
        graphicsView->setMinimumSize(QSize(500, 500));

        verticalLayout_2->addWidget(graphicsView);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_6);

        showMeasuredValues = new QCheckBox(horizontalLayoutWidget);
        showMeasuredValues->setObjectName(QString::fromUtf8("showMeasuredValues"));
        showMeasuredValues->setChecked(true);

        horizontalLayout_7->addWidget(showMeasuredValues);

        horizontalSpacer_13 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_13);

        showModelledValues = new QCheckBox(horizontalLayoutWidget);
        showModelledValues->setObjectName(QString::fromUtf8("showModelledValues"));
        showModelledValues->setChecked(true);

        horizontalLayout_7->addWidget(showModelledValues);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_11);


        verticalLayout_2->addLayout(horizontalLayout_7);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_6);


        horizontalLayout->addLayout(verticalLayout_2);

        horizontalSpacer_4 = new QSpacerItem(20, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_8 = new QLabel(horizontalLayoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMinimumSize(QSize(150, 0));
        label_8->setMaximumSize(QSize(150, 16777215));

        verticalLayout_3->addWidget(label_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_10 = new QLabel(horizontalLayoutWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_9->addWidget(label_10);

        modelSelection = new QComboBox(horizontalLayoutWidget);
        modelSelection->setObjectName(QString::fromUtf8("modelSelection"));
        modelSelection->setMinimumSize(QSize(130, 0));

        horizontalLayout_9->addWidget(modelSelection);


        verticalLayout_3->addLayout(horizontalLayout_9);

        fitModelButton = new QPushButton(horizontalLayoutWidget);
        fitModelButton->setObjectName(QString::fromUtf8("fitModelButton"));

        verticalLayout_3->addWidget(fitModelButton);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        par1Label = new QLabel(horizontalLayoutWidget);
        par1Label->setObjectName(QString::fromUtf8("par1Label"));

        horizontalLayout_8->addWidget(par1Label);

        par1Value = new QLineEdit(horizontalLayoutWidget);
        par1Value->setObjectName(QString::fromUtf8("par1Value"));
        par1Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par1Value->sizePolicy().hasHeightForWidth());
        par1Value->setSizePolicy(sizePolicy1);
        par1Value->setMinimumSize(QSize(60, 0));
        par1Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_8->addWidget(par1Value);


        verticalLayout_3->addLayout(horizontalLayout_8);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        par2Label = new QLabel(horizontalLayoutWidget);
        par2Label->setObjectName(QString::fromUtf8("par2Label"));

        horizontalLayout_10->addWidget(par2Label);

        par2Value = new QLineEdit(horizontalLayoutWidget);
        par2Value->setObjectName(QString::fromUtf8("par2Value"));
        par2Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par2Value->sizePolicy().hasHeightForWidth());
        par2Value->setSizePolicy(sizePolicy1);
        par2Value->setMinimumSize(QSize(60, 0));
        par2Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_10->addWidget(par2Value);


        verticalLayout_3->addLayout(horizontalLayout_10);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        par3Label = new QLabel(horizontalLayoutWidget);
        par3Label->setObjectName(QString::fromUtf8("par3Label"));

        horizontalLayout_13->addWidget(par3Label);

        par3Value = new QLineEdit(horizontalLayoutWidget);
        par3Value->setObjectName(QString::fromUtf8("par3Value"));
        par3Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par3Value->sizePolicy().hasHeightForWidth());
        par3Value->setSizePolicy(sizePolicy1);
        par3Value->setMinimumSize(QSize(60, 0));
        par3Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_13->addWidget(par3Value);


        verticalLayout_3->addLayout(horizontalLayout_13);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        par4Label = new QLabel(horizontalLayoutWidget);
        par4Label->setObjectName(QString::fromUtf8("par4Label"));

        horizontalLayout_18->addWidget(par4Label);

        par4Value = new QLineEdit(horizontalLayoutWidget);
        par4Value->setObjectName(QString::fromUtf8("par4Value"));
        par4Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par4Value->sizePolicy().hasHeightForWidth());
        par4Value->setSizePolicy(sizePolicy1);
        par4Value->setMinimumSize(QSize(60, 0));
        par4Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_18->addWidget(par4Value);


        verticalLayout_3->addLayout(horizontalLayout_18);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        par5Label = new QLabel(horizontalLayoutWidget);
        par5Label->setObjectName(QString::fromUtf8("par5Label"));

        horizontalLayout_19->addWidget(par5Label);

        par5Value = new QLineEdit(horizontalLayoutWidget);
        par5Value->setObjectName(QString::fromUtf8("par5Value"));
        par5Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par5Value->sizePolicy().hasHeightForWidth());
        par5Value->setSizePolicy(sizePolicy1);
        par5Value->setMinimumSize(QSize(60, 0));
        par5Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_19->addWidget(par5Value);


        verticalLayout_3->addLayout(horizontalLayout_19);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
        par6Label = new QLabel(horizontalLayoutWidget);
        par6Label->setObjectName(QString::fromUtf8("par6Label"));

        horizontalLayout_20->addWidget(par6Label);

        par6Value = new QLineEdit(horizontalLayoutWidget);
        par6Value->setObjectName(QString::fromUtf8("par6Value"));
        par6Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par6Value->sizePolicy().hasHeightForWidth());
        par6Value->setSizePolicy(sizePolicy1);
        par6Value->setMinimumSize(QSize(60, 0));
        par6Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_20->addWidget(par6Value);


        verticalLayout_3->addLayout(horizontalLayout_20);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
        par7Label = new QLabel(horizontalLayoutWidget);
        par7Label->setObjectName(QString::fromUtf8("par7Label"));

        horizontalLayout_21->addWidget(par7Label);

        par7Value = new QLineEdit(horizontalLayoutWidget);
        par7Value->setObjectName(QString::fromUtf8("par7Value"));
        par7Value->setEnabled(false);
        sizePolicy1.setHeightForWidth(par7Value->sizePolicy().hasHeightForWidth());
        par7Value->setSizePolicy(sizePolicy1);
        par7Value->setMinimumSize(QSize(60, 0));
        par7Value->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_21->addWidget(par7Value);


        verticalLayout_3->addLayout(horizontalLayout_21);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_5);


        horizontalLayout->addLayout(verticalLayout_3);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_5);

        ValveAnalyser->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ValveAnalyser);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1119, 22));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        ValveAnalyser->setMenuBar(menubar);
        statusbar = new QStatusBar(ValveAnalyser);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        ValveAnalyser->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionPrint);
        menuFile->addAction(actionQuit);
        menuFile->addSeparator();
        menuFile->addAction(actionOptions);
        menuHelp->addAction(actionContents);
        menuHelp->addAction(actionAbout);

        retranslateUi(ValveAnalyser);

        QMetaObject::connectSlotsByName(ValveAnalyser);
    } // setupUi

    void retranslateUi(QMainWindow *ValveAnalyser)
    {
        ValveAnalyser->setWindowTitle(QCoreApplication::translate("ValveAnalyser", "Wizard Valve Analyser", nullptr));
        actionPrint->setText(QCoreApplication::translate("ValveAnalyser", "Print", nullptr));
#if QT_CONFIG(shortcut)
        actionPrint->setShortcut(QCoreApplication::translate("ValveAnalyser", "Ctrl+P", nullptr));
#endif // QT_CONFIG(shortcut)
        actionQuit->setText(QCoreApplication::translate("ValveAnalyser", "Quit", nullptr));
#if QT_CONFIG(shortcut)
        actionQuit->setShortcut(QCoreApplication::translate("ValveAnalyser", "Ctrl+Q", nullptr));
#endif // QT_CONFIG(shortcut)
        actionOptions->setText(QCoreApplication::translate("ValveAnalyser", "Options...", nullptr));
        actionContents->setText(QCoreApplication::translate("ValveAnalyser", "About...", nullptr));
        actionAbout->setText(QCoreApplication::translate("ValveAnalyser", "Analyser Help", nullptr));
        pushButton_2->setText(QCoreApplication::translate("ValveAnalyser", "Load Template...", nullptr));
        pushButton->setText(QCoreApplication::translate("ValveAnalyser", "Save Template...", nullptr));
        label_7->setText(QCoreApplication::translate("ValveAnalyser", "Device Name:", nullptr));
        deviceTypeLabel->setText(QCoreApplication::translate("ValveAnalyser", "Device Type: ", nullptr));
        label_4->setText(QCoreApplication::translate("ValveAnalyser", "Test Type:", nullptr));
        heaterLabel->setText(QCoreApplication::translate("ValveAnalyser", "Heater Voltage:", nullptr));
        label->setText(QCoreApplication::translate("ValveAnalyser", "Start", nullptr));
        label_2->setText(QCoreApplication::translate("ValveAnalyser", "Stop", nullptr));
        label_3->setText(QCoreApplication::translate("ValveAnalyser", "Step", nullptr));
        anodeLabel->setText(QCoreApplication::translate("ValveAnalyser", "Anode Voltage:", nullptr));
        gridLabel->setText(QCoreApplication::translate("ValveAnalyser", "-ve Grid Voltage:", nullptr));
        screenLabel->setText(QCoreApplication::translate("ValveAnalyser", "Screen Voltage:", nullptr));
        label_5->setText(QCoreApplication::translate("ValveAnalyser", "Max Ia (mA):", nullptr));
        label_6->setText(QCoreApplication::translate("ValveAnalyser", "Max P (W):", nullptr));
        heaterButton->setText(QCoreApplication::translate("ValveAnalyser", "Heater", nullptr));
        heaterVLabel->setText(QCoreApplication::translate("ValveAnalyser", "Heater Voltage (V)", nullptr));
        heaterILabel->setText(QCoreApplication::translate("ValveAnalyser", "Heater Current (mA)", nullptr));
        runButton->setText(QCoreApplication::translate("ValveAnalyser", "Run Test", nullptr));
        plotTitle->setText(QCoreApplication::translate("ValveAnalyser", "No valid test data to display", nullptr));
        showMeasuredValues->setText(QCoreApplication::translate("ValveAnalyser", "Show Measured Values", nullptr));
        showModelledValues->setText(QCoreApplication::translate("ValveAnalyser", "Show Modelled Values", nullptr));
        label_8->setText(QCoreApplication::translate("ValveAnalyser", "Device Modelling", nullptr));
        label_10->setText(QCoreApplication::translate("ValveAnalyser", "Model:", nullptr));
        fitModelButton->setText(QCoreApplication::translate("ValveAnalyser", "Fit Model", nullptr));
        par1Label->setText(QCoreApplication::translate("ValveAnalyser", "Mu:", nullptr));
        par2Label->setText(QCoreApplication::translate("ValveAnalyser", "Kg:", nullptr));
        par3Label->setText(QCoreApplication::translate("ValveAnalyser", "Alpha:", nullptr));
        par4Label->setText(QCoreApplication::translate("ValveAnalyser", "Vct:", nullptr));
        par5Label->setText(QCoreApplication::translate("ValveAnalyser", "Kp:", nullptr));
        par6Label->setText(QCoreApplication::translate("ValveAnalyser", "Kvb:", nullptr));
        par7Label->setText(QCoreApplication::translate("ValveAnalyser", "Kvb2:", nullptr));
        menuFile->setTitle(QCoreApplication::translate("ValveAnalyser", "File", nullptr));
        menuHelp->setTitle(QCoreApplication::translate("ValveAnalyser", "Help", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ValveAnalyser: public Ui_ValveAnalyser {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VALVEANALYSER_H
