/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *btnStart;
    QPushButton *btnStop;
    QLabel *lblStatus;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLineEdit *editZSet;
    QLineEdit *editYSet;
    QLineEdit *editXSet;
    QWidget *widget3D;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QSlider *sliderRoll;
    QSlider *sliderYaw;
    QLabel *lblRollVal;
    QLabel *lblPitchVal;
    QLabel *lblYawVal;
    QCheckBox *chkEnableAtt;
    QSlider *sliderPitch;
    QPushButton *btnHeart;
    QPushButton *pushButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1103, 642);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        btnStart = new QPushButton(centralwidget);
        btnStart->setObjectName(QString::fromUtf8("btnStart"));
        btnStart->setGeometry(QRect(10, 70, 89, 25));
        btnStop = new QPushButton(centralwidget);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setGeometry(QRect(10, 140, 89, 25));
        lblStatus = new QLabel(centralwidget);
        lblStatus->setObjectName(QString::fromUtf8("lblStatus"));
        lblStatus->setGeometry(QRect(160, 90, 181, 41));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(210, 220, 67, 17));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(210, 190, 67, 17));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(210, 160, 67, 17));
        editZSet = new QLineEdit(centralwidget);
        editZSet->setObjectName(QString::fromUtf8("editZSet"));
        editZSet->setGeometry(QRect(270, 220, 113, 25));
        editYSet = new QLineEdit(centralwidget);
        editYSet->setObjectName(QString::fromUtf8("editYSet"));
        editYSet->setGeometry(QRect(270, 190, 113, 25));
        editXSet = new QLineEdit(centralwidget);
        editXSet->setObjectName(QString::fromUtf8("editXSet"));
        editXSet->setGeometry(QRect(270, 160, 113, 25));
        widget3D = new QWidget(centralwidget);
        widget3D->setObjectName(QString::fromUtf8("widget3D"));
        widget3D->setGeometry(QRect(519, 9, 571, 391));
        verticalLayoutWidget = new QWidget(widget3D);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 0, 571, 391));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        sliderRoll = new QSlider(centralwidget);
        sliderRoll->setObjectName(QString::fromUtf8("sliderRoll"));
        sliderRoll->setGeometry(QRect(100, 370, 231, 16));
        sliderRoll->setMinimum(-45);
        sliderRoll->setMaximum(45);
        sliderRoll->setSingleStep(1);
        sliderRoll->setValue(0);
        sliderRoll->setOrientation(Qt::Horizontal);
        sliderYaw = new QSlider(centralwidget);
        sliderYaw->setObjectName(QString::fromUtf8("sliderYaw"));
        sliderYaw->setGeometry(QRect(100, 440, 231, 16));
        sliderYaw->setMinimum(-90);
        sliderYaw->setMaximum(90);
        sliderYaw->setSingleStep(1);
        sliderYaw->setOrientation(Qt::Horizontal);
        lblRollVal = new QLabel(centralwidget);
        lblRollVal->setObjectName(QString::fromUtf8("lblRollVal"));
        lblRollVal->setGeometry(QRect(340, 370, 67, 17));
        lblPitchVal = new QLabel(centralwidget);
        lblPitchVal->setObjectName(QString::fromUtf8("lblPitchVal"));
        lblPitchVal->setGeometry(QRect(30, 510, 67, 17));
        lblYawVal = new QLabel(centralwidget);
        lblYawVal->setObjectName(QString::fromUtf8("lblYawVal"));
        lblYawVal->setGeometry(QRect(340, 440, 67, 17));
        chkEnableAtt = new QCheckBox(centralwidget);
        chkEnableAtt->setObjectName(QString::fromUtf8("chkEnableAtt"));
        chkEnableAtt->setGeometry(QRect(200, 310, 92, 23));
        sliderPitch = new QSlider(centralwidget);
        sliderPitch->setObjectName(QString::fromUtf8("sliderPitch"));
        sliderPitch->setGeometry(QRect(50, 330, 16, 160));
        sliderPitch->setMinimum(-45);
        sliderPitch->setMaximum(45);
        sliderPitch->setOrientation(Qt::Vertical);
        btnHeart = new QPushButton(centralwidget);
        btnHeart->setObjectName(QString::fromUtf8("btnHeart"));
        btnHeart->setGeometry(QRect(10, 210, 120, 25));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(50, 260, 111, 21));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1103, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        btnStart->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        btnStop->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        lblStatus->setText(QCoreApplication::translate("MainWindow", "Status", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Z_set", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Y_set", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "X_set", nullptr));
        lblRollVal->setText(QCoreApplication::translate("MainWindow", "Roll_set", nullptr));
        lblPitchVal->setText(QCoreApplication::translate("MainWindow", "Pitch_set", nullptr));
        lblYawVal->setText(QCoreApplication::translate("MainWindow", "Yaw_set", nullptr));
        chkEnableAtt->setText(QCoreApplication::translate("MainWindow", "CheckBox", nullptr));
        btnHeart->setText(QCoreApplication::translate("MainWindow", "Heart Trajectory", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Change mode", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
