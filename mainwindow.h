#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QTimer *timer;

private slots:
    void initFrameProc();
    void initCams();
    void openCamera();
    void updateFrame();
    void on_pushButton_clicked();
    void on_RadiusSlider_valueChanged(int value);
    void on_ThresholdSlider_valueChanged(int value);
    void on_CloseCameras_clicked();
};
#endif // MAINWINDOW_H
