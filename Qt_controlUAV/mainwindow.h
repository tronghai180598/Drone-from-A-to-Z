#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include <QVector3D>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// Forward declarations
namespace Qt3DCore {
    class QEntity;
    class QAttribute;
    class QBuffer;
    class QGeometry;
}
namespace Qt3DRender {
    class QGeometryRenderer;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onRollChanged(int deg);
    void onPitchChanged(int deg);
    void onYawChanged(int deg);

    void sendRcTick();

private:
    Ui::MainWindow *ui;
    bool writeCommand(const QString &cmd);

    // Attitude commands (rad)
    float rollCmdRad_{0.0f};
    float pitchCmdRad_{0.0f};
    float yawCmdRad_{0.0f};

    bool armed_{false};
    bool stickMode_{true};  // true = stick mode, false = auto mode
    QTimer rcTimer_;
    
    // Trail visualization
    Qt3DCore::QEntity *trailEntity_{nullptr};
    Qt3DRender::QGeometryRenderer *trailRenderer_{nullptr};
    QVector<QVector3D> trailPoints_;
    static constexpr int MAX_TRAIL_POINTS = 2000;
    void updateTrailGeometry();
    
    // Reference heart trajectory visualization
    Qt3DCore::QEntity *referenceHeartEntity_{nullptr};
    Qt3DRender::QGeometryRenderer *referenceHeartRenderer_{nullptr};
    QVector<QVector3D> referenceHeartPoints_;
    void generateReferenceHeartTrajectory(float x0, float y0, float z);
    void updateReferenceHeartGeometry();
};

#endif // MAINWINDOW_H
