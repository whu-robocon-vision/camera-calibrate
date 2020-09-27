#ifndef MYREALSENSE_H
#define MYREALSENSE_H
#include "common.h"

enum RSMode{
    COLOR_MODE,
    DEPTH_MODE,
    INFRA1_MODE,
    INFRA2_MODE
};

enum RSState{
    WORK,
    STOP,
    PAUSE
};

enum eCaliPos{
    UL,
    UR,
    DR,
    DL
};

namespace rs2
{
class MyRealsense : public QThread
{
    Q_OBJECT

signals:
    void frameOK();
    void caliOK(bool isOK);

public:
    MyRealsense(QObject *parent);
    ~MyRealsense();

    RSMode getMode() {return this->mode;}
    void setMode(RSMode mode) {this->mode = mode;}
    bool open(QLabel *label);
    bool open(QLabel *lab_cover, QLabel *lab_fps);
    RSState getState() {return this->state;}
    bool close(QLabel *label);

    bool reopen();
    void pause() {this->state = PAUSE;}

    int getFps() {return this->fps_s;}
    void setFps(int fps) {this->fps_s = fps;}
    int getFpsNow();

    void updateImg();
    cv::Mat color_img;
    cv::Mat depth_img;

    QLabel *getCover() {return this->cover;}
    QLabel *getLabFps() {return this->lab_fps;}
    float getDist(int x, int y);
    cv::Point3f getWorldCoord(int x, int y);
    void calibrate(int n_row, int n_col);
    std::vector<cv::Point2f> getCorners() {return this->corners;}
    int getCaliRows() {return this->n_rows;}
    int getCaliCols() {return this->n_cols;}
    void setCaliPos(eCaliPos pos) {this->cali_pos = pos;}
    eCaliPos getCaliPos() {return this->cali_pos;}

    rs2_intrinsics color_intrin;
    rs2_intrinsics depth_intrin;
    cv::Mat extrin_rmat;
    cv::Mat extrin_tmat;
    void genExtrin();
    bool is_x_inv;
    bool is_y_inv;
    bool is_z_inv;
    float cali_x;
    float cali_y;
    float cali_z;
    float cali_step;

    frame color_frame;
    frame depth_frame;
private:
    std::vector<cv::Point3f> getMualWorldCoor();
    cv::Point3f camera2world(cv::Point3f point);

    /* device */
    context ctx;
    device dev;
    depth_sensor *dep_sensor;

    /* calibrate */
    std::vector<cv::Point2f> corners;
    int n_rows;
    int n_cols;
    eCaliPos cali_pos;

    RSState state;
    RSMode mode;

    pipeline pipe;
    config cfg;

    QLabel *cover;
    QLabel *lab_fps;
    size_t width;
    size_t height;

    time_t fps_start;
    time_t fps_end;
    int fps_n; //fps actual
    int fps_s; //fps set
protected:
    void run();
};
}


#endif // MYREALSENSE_H
