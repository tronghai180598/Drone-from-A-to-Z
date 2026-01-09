#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QTextStream>
#include <QMessageBox>

static const char* CMD_FILE = "/tmp/uav_cmd";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // Initialize attitude mode
    attitudeMode_ = "pid";
    // Button shows the mode that will be switched TO (not current mode)
    ui->btnAttitudeMode->setText("PDPI");
    
    // Toggle button: switch between PID and PDPI
    // Button text shows what mode you will switch TO when clicked
    connect(ui->btnAttitudeMode, &QPushButton::clicked, this, [this]() {
        if (attitudeMode_ == "pid") {
            // Currently PID, switch to PDPI
            attitudeMode_ = "pdpi";
            writeCommand("attitude pdpi");
            ui->btnAttitudeMode->setText("PID");  // Next click will switch to PID
        } else {
            // Currently PDPI, switch to PID
            attitudeMode_ = "pid";
            writeCommand("attitude pid");
            ui->btnAttitudeMode->setText("PDPI");  // Next click will switch to PDPI
        }
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::writeCommand(const QString &cmd)
{
    QFile f(CMD_FILE);

    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QMessageBox::critical(this, "Error",
                              QString("Cannot open %1\nReason: %2")
                              .arg(CMD_FILE, f.errorString()));
        return false;
    }

    QTextStream out(&f);
    out << cmd << "\n";
    out.flush();
    f.flush();
    f.close();
    return true;
}
