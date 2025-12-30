#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDoubleValidator>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QDebug>
#include <QHostAddress>
#include <QUdpSocket>
#include <QColor>

// Qt3D
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QDirectionalLight>
#include <QtMath>
#include <QVector3D>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static float degToRad(float deg) { return deg * float(M_PI) / 180.0f; }
static float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static const char* CMD_FILE = "/tmp/uav_cmd";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // =========================
    // 3D VIEW (Qt3D)
    // =========================
    auto *view = new Qt3DExtras::Qt3DWindow();
    auto *container = QWidget::createWindowContainer(view, ui->widget3D);
    container->setMinimumSize(QSize(300, 300));
    container->setFocusPolicy(Qt::StrongFocus);

    if (!ui->widget3D->layout()) {
        auto *lay = new QVBoxLayout(ui->widget3D);
        lay->setContentsMargins(0,0,0,0);
    }
    ui->widget3D->layout()->addWidget(container);

    auto *root = new Qt3DCore::QEntity();

    // Light
    auto *lightEntity = new Qt3DCore::QEntity(root);
    auto *light = new Qt3DRender::QDirectionalLight(lightEntity);
    light->setIntensity(1.2f);
    light->setWorldDirection(QVector3D(-1.0f, -1.0f, -1.0f));
    lightEntity->addComponent(light);

    view->setRootEntity(root);

    // Camera - fixed top-down view for better trail viewing (doesn't follow UAV)
    auto *cam = view->camera();
    cam->lens()->setPerspectiveProjection(60.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cam->setUpVector(QVector3D(0,1,0)); // Y-up for top view
    cam->setPosition(QVector3D(0, 0, 15)); // Top-down view, high above, fixed position
    cam->setViewCenter(QVector3D(0,0,2)); // Look at center, fixed (not following UAV)

    auto *camCtrl = new Qt3DExtras::QOrbitCameraController(root);
    camCtrl->setCamera(cam);

    // =========================
    // TRAIL VISUALIZATION
    // =========================
    trailEntity_ = new Qt3DCore::QEntity(root);
    
    // Create geometry for line strip
    auto *trailGeometry = new Qt3DRender::QGeometry(trailEntity_);
    
    // Position buffer
    auto *positionBuffer = new Qt3DRender::QBuffer(trailEntity_);
    positionBuffer->setData(QByteArray());
    
    auto *positionAttribute = new Qt3DRender::QAttribute(trailEntity_);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(positionBuffer);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(0);
    trailGeometry->addAttribute(positionAttribute);
    
    // Create geometry renderer
    trailRenderer_ = new Qt3DRender::QGeometryRenderer(trailEntity_);
    trailRenderer_->setGeometry(trailGeometry);
    trailRenderer_->setPrimitiveType(Qt3DRender::QGeometryRenderer::LineStrip);
    
    // Material for trail (bright red, emissive for better visibility)
    auto *trailMat = new Qt3DExtras::QPhongMaterial(trailEntity_);
    trailMat->setDiffuse(QColor(255, 0, 0)); // Red
    trailMat->setAmbient(QColor(255, 0, 0));
    trailMat->setSpecular(QColor(255, 0, 0));
    trailMat->setShininess(0.0f); // No specular highlight
    
    trailEntity_->addComponent(trailRenderer_);
    trailEntity_->addComponent(trailMat);
    
    trailPoints_.clear();
    
    qDebug() << "[TRAIL] Trail entity created and initialized, waiting for UDP data...";

    // =========================
    // UDP receiver (position only for trail)
    // =========================
    auto *sock = new QUdpSocket(this);
    sock->bind(QHostAddress::AnyIPv4, 5005,
               QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    connect(sock, &QUdpSocket::readyRead, this, [=]() mutable {
        while (sock->hasPendingDatagrams()) {
            QByteArray d;
            d.resize(int(sock->pendingDatagramSize()));
            sock->readDatagram(d.data(), d.size());

            // Expect: t x y z qx qy qz qw
            QList<QByteArray> p = d.simplified().split(' ');
            if (p.size() < 8) {
                qDebug() << "[TRAIL] Invalid packet size:" << p.size();
                continue;
            }

            bool ok = true;
            float x = p[1].toFloat(&ok); if(!ok) continue;
            float y = p[2].toFloat(&ok); if(!ok) continue;
            float z = p[3].toFloat(&ok); if(!ok) continue;
            
            // Only use position for trail, ignore quaternion
            QVector3D pos(x, y, z);
            
            // Only add point if it's far enough from the last point (to avoid too many points)
            static QVector3D lastPos(0, 0, 0);
            static bool firstPoint = true;
            
            if (firstPoint || (pos - lastPos).length() > 0.05f) { // Add point if > 5cm away
                trailPoints_.append(pos);
                lastPos = pos;
                firstPoint = false;
                
                // Debug: print first few points
                static int debugCount = 0;
                if (debugCount < 5) {
                    qDebug() << "[TRAIL] Point" << debugCount << ":" << pos;
                    debugCount++;
                }
                
                // Limit trail length (keep all points, don't remove old ones)
                // Only limit if we exceed MAX_TRAIL_POINTS significantly
                if (trailPoints_.size() > MAX_TRAIL_POINTS * 1.5f) {
                    // Remove oldest 25% of points
                    int removeCount = trailPoints_.size() / 4;
                    trailPoints_.remove(0, removeCount);
                }
                
                // Update trail geometry if we have at least 2 points
                if (trailPoints_.size() >= 2) {
                    updateTrailGeometry();
                    // Debug: print when updating
                    if (trailPoints_.size() % 100 == 0) {
                        qDebug() << "[TRAIL] Updated with" << trailPoints_.size() << "points";
                    }
                }
            }
        }
    });

    // =========================
    // Slider setup (phải setup ở đây, không phải trong UDP lambda)
    // =========================
    ui->sliderRoll->setRange(-45, 45);
    ui->sliderPitch->setRange(-45, 45);
        ui->sliderYaw->setRange(-90, 90);

        ui->sliderRoll->setValue(0);
        ui->sliderPitch->setValue(0);
        ui->sliderYaw->setValue(0);

        // Connect slider
        connect(ui->sliderRoll,  &QSlider::valueChanged, this, &MainWindow::onRollChanged);
        connect(ui->sliderPitch, &QSlider::valueChanged, this, &MainWindow::onPitchChanged);
        connect(ui->sliderYaw,   &QSlider::valueChanged, this, &MainWindow::onYawChanged);
    // Roll: thả ra về 0
    connect(ui->sliderRoll, &QSlider::sliderReleased, this, [this]() {
        ui->sliderRoll->setValue(0);
    });

    // Pitch: thả ra về 0
    connect(ui->sliderPitch, &QSlider::sliderReleased, this, [this]() {
        ui->sliderPitch->setValue(0);
    });

    // Yaw: thả ra về 0 (nếu bạn cũng muốn yaw tự về 0)
    connect(ui->sliderYaw, &QSlider::sliderReleased, this, [this]() {
        ui->sliderYaw->setValue(0);
    });
        // Timer gửi attitude (50 Hz)
        rcTimer_.setInterval(20);
        connect(&rcTimer_, &QTimer::timeout, this, &MainWindow::sendRcTick);
        rcTimer_.start();
    // =========================
    // UI: điều khiển X, Y, Z
    // =========================

    // Validator cho X, Y, Z
    auto *valX = new QDoubleValidator(-100.0, 100.0, 3, this);
    valX->setNotation(QDoubleValidator::StandardNotation);
    ui->editXSet->setValidator(valX);

    auto *valY = new QDoubleValidator(-100.0, 100.0, 3, this);
    valY->setNotation(QDoubleValidator::StandardNotation);
    ui->editYSet->setValidator(valY);

    auto *valZ = new QDoubleValidator(-100.0, 100.0, 3, this);
    valZ->setNotation(QDoubleValidator::StandardNotation);
    ui->editZSet->setValidator(valZ);

    // Set X khi nhập xong
    connect(ui->editXSet, &QLineEdit::editingFinished, this, [this]() {
        bool ok=false;
        double x = ui->editXSet->text().toDouble(&ok);
        if (!ok) return;

        writeCommand(QString("set x %1").arg(x, 0, 'f', 3));
        ui->lblStatus->setText(QString("Status: X_set = %1").arg(x, 0, 'f', 3));
    });

    // Set Y khi nhập xong
    connect(ui->editYSet, &QLineEdit::editingFinished, this, [this]() {
        bool ok=false;
        double y = ui->editYSet->text().toDouble(&ok);
        if (!ok) return;

        writeCommand(QString("set y %1").arg(y, 0, 'f', 3));
        ui->lblStatus->setText(QString("Status: Y_set = %1").arg(y, 0, 'f', 3));
    });

    // Set Z khi nhập xong
    connect(ui->editZSet, &QLineEdit::editingFinished, this, [this]() {
        bool ok=false;
        double z = ui->editZSet->text().toDouble(&ok);
        if (!ok) return;

        writeCommand(QString("set z %1").arg(z, 0, 'f', 3));
        ui->lblStatus->setText(QString("Status: Z_set = %1").arg(z, 0, 'f', 3));
    });

    // Start: set x, y, z (nếu có) rồi arm
    connect(ui->btnStart, &QPushButton::clicked, this, [this]() {
        bool ok=false;
        double x = ui->editXSet->text().toDouble(&ok);
        if (ok) writeCommand(QString("set x %1").arg(x, 0, 'f', 3));
        
        double y = ui->editYSet->text().toDouble(&ok);
        if (ok) writeCommand(QString("set y %1").arg(y, 0, 'f', 3));
        
        double z = ui->editZSet->text().toDouble(&ok);
        if (ok) writeCommand(QString("set z %1").arg(z, 0, 'f', 3));

        writeCommand("arm");
        armed_ = true;  // FIX: Set armed_ = true để slider có thể gửi command
        ui->lblStatus->setText("Status: ARMED");
    });

    // Stop: disarm
    connect(ui->btnStop, &QPushButton::clicked, this, [this]() {
        if (writeCommand("disarm")) {
            armed_ = false;  // FIX: Set armed_ = false khi disarm
            ui->lblStatus->setText("Status: DISARMED");
        }
    });

    // Initialize checkbox for stick mode
    ui->chkEnableAtt->setText("Stick Mode");
    ui->chkEnableAtt->setChecked(true);  // Default: stick mode
    stickMode_ = true;
    writeCommand("mode stick");  // Set initial mode
    
    // Change mode button: toggle between stick and auto
    connect(ui->pushButton, &QPushButton::clicked, this, [this]() {
        stickMode_ = !stickMode_;
        if (stickMode_) {
            writeCommand("mode stick");
            ui->chkEnableAtt->setChecked(true);
            ui->chkEnableAtt->setText("Stick Mode");
            ui->lblStatus->setText("Status: Stick Mode");
        } else {
            writeCommand("mode auto");
            ui->chkEnableAtt->setChecked(false);
            ui->chkEnableAtt->setText("Auto Mode");
            ui->lblStatus->setText("Status: Auto Mode");
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
void MainWindow::onRollChanged(int deg)
{
    rollCmdRad_ = clampf(degToRad(float(deg)), -0.78f, 0.78f);
    ui->lblRollVal->setText(QString::number(deg) + " deg");
}

void MainWindow::onPitchChanged(int deg)
{
    pitchCmdRad_ = clampf(degToRad(float(deg)), -0.78f, 0.78f);
    ui->lblPitchVal->setText(QString::number(deg) + " deg");
}

void MainWindow::onYawChanged(int deg)
{
    yawCmdRad_ = degToRad(float(deg)); // yaw cho phép rộng hơn
    ui->lblYawVal->setText(QString::number(deg) + " deg");
}
void MainWindow::sendRcTick()
{
    // an toàn: phải bật checkbox
    if (!ui->chkEnableAtt->isChecked())
        return;

    // an toàn: chỉ gửi khi đã arm
    if (!armed_)
        return;

    // Gửi lệnh RC cho cả 2 chế độ (stick và auto)
    // Slider luôn cập nhật roll_set, pitch_set, yaw_set
    QString cmd = QString("rc %1 %2 %3")
                      .arg(rollCmdRad_,  0, 'f', 6)
                      .arg(pitchCmdRad_, 0, 'f', 6)
                      .arg(yawCmdRad_,   0, 'f', 6);

    writeCommand(cmd);
}

void MainWindow::updateTrailGeometry()
{
    if (!trailRenderer_ || trailPoints_.size() < 2) return;
    
    // Get geometry and position attribute
    auto *geometry = trailRenderer_->geometry();
    if (!geometry) return;
    
    auto attributes = geometry->attributes();
    if (attributes.isEmpty()) return;
    
    auto *positionAttribute = attributes.first();
    if (!positionAttribute) return;
    
    auto *buffer = positionAttribute->buffer();
    if (!buffer) return;
    
    // Create vertex data
    QByteArray vertexData;
    vertexData.resize(trailPoints_.size() * 3 * sizeof(float));
    float *rawVertexArray = reinterpret_cast<float*>(vertexData.data());
    
    int idx = 0;
    for (const QVector3D &pt : trailPoints_) {
        rawVertexArray[idx++] = pt.x();
        rawVertexArray[idx++] = pt.y();
        rawVertexArray[idx++] = pt.z();
    }
    
    // Update buffer
    buffer->setData(vertexData);
    positionAttribute->setCount(trailPoints_.size());
    
    qDebug() << "[TRAIL] Geometry updated: count=" << trailPoints_.size();
}
