#include "myrealsense.h"

namespace rs2 {
MyRealsense::MyRealsense(QObject *parent) : QThread(parent)
{
    state = STOP;
    mode = COLOR_MODE;
    fps_start = fps_end = 0;
    lab_fps = cover = NULL;
    fps_s = 60;

    extrin_rmat = cv::Mat(cv::Matx33f(0.99968761, 0.023478003, 0.0085698199,
                            -0.023162002, 0.99910909, -0.035277449,
                            -0.0093904296, 0.035067935, 0.99934083));
    extrin_tmat = cv::Mat(cv::Matx31f(120.526634,
                            263.11539,
                            -447.59726));
}

MyRealsense::~MyRealsense()
{
    this->close(cover);
    delete this;
}

bool MyRealsense::open(QLabel *label)
{
    DBG(GREEN "<open>" NONE);
    if (state == WORK)
        return true;
    try {
        this->cover = label;
        this->start();
    } catch (rs2::error &e) {
        DBG(RED "<rs2 error>" NONE "%s", e.what());
        return false;
    } catch (std::exception &e) {
        DBG(RED "<std error>" NONE "%s", e.what());
        return false;
    }
    return true;
}

bool MyRealsense::open(QLabel *lab_cover, QLabel *lab_fps)
{
    DBG(GREEN "<open>" NONE);
    if (state == WORK)
        return true;
    try {
        this->cover = lab_cover;
        this->lab_fps = lab_fps;
        this->start();
    } catch (rs2::error &e) {
        DBG(RED "<rs2 error>" NONE "%s", e.what());
        return false;
    } catch (std::exception &e) {
        DBG(RED "<std error>" NONE "%s", e.what());
        return false;
    }
    return true;
}

bool MyRealsense::close(QLabel *label)
{
    if (state == STOP)
        return true;
    try {
        state = STOP;
        pipe.stop();
        label->clear();
        DBG(GREEN "<camera> : " NONE "camera closed");
    } catch (error &e) {
        DBG(RED "<rs2 error> : " NONE "%s", e.what());
        return false;
    } catch (std::exception &e) {
        DBG(RED "<std error> : " NONE "%s", e.what());
    }
    return true;
}

bool MyRealsense::reopen()
{
    if (state == PAUSE || state == WORK) {
        state = WORK;
        return true;
    } else {
        DBG(RED "<reopen()> : " NONE "device not start");
        return false;
    }
}

int MyRealsense::getFpsNow()
{
    fps_end = clock();
    fps_n = CLOCKS_PER_SEC / (fps_end - fps_start);
    fps_start = clock();
    return fps_n;
}

void MyRealsense::updateImg()
{
    QImage image;
    static int counter = 0;
    cv::Mat color_img, depth_img;

    switch (this->getMode()) {
    case COLOR_MODE:
        cv::cvtColor(this->color_img, color_img, cv::COLOR_BGR2RGB);
        image = QImage((uchar *)(color_img.data), color_img.cols,
                                 color_img.rows, color_img.step, QImage::Format_RGB888);
        break;
    case DEPTH_MODE:
        this->depth_img.copyTo(depth_img);
        depth_img.convertTo(depth_img, CV_8UC1);
        cv::applyColorMap(depth_img, depth_img, cv::COLORMAP_JET);
        image = QImage((uchar *)(depth_img.data), depth_img.cols,
                                 depth_img.rows, depth_img.step, QImage::Format_RGB888);
        break;
    default:
        cv::cvtColor(this->color_img, color_img, cv::COLOR_BGR2RGB);
        image = QImage((uchar *)(color_img.data), color_img.cols,
                                 color_img.rows, color_img.step, QImage::Format_RGB888);
        break;
    }
    this->getCover()->setPixmap(QPixmap::fromImage(image));
    this->getCover()->show();
    int fps = this->getFpsNow();

    if (counter++ == this->getFps() / 10) {
        counter = 0;
        this->getLabFps()->setNum(fps);
    }
}

float MyRealsense::getDist(int x, int y)
{
    float scale = dep_sensor->get_depth_scale();
    // DBG(BLUE "<sensor> : " NONE "depth scale : %f", scale);
    uint16_t img_val = depth_img.at<uint16_t>(y, x);
    return img_val * scale * 1000;
}

cv::Point3f MyRealsense::getWorldCoord(int x, int y)
{
    float pixel[2] = {(float)x, (float)y};
    float point[3] = {0};
    float dist = getDist(x, y);
    rs2_deproject_pixel_to_point(point, &color_intrin, pixel, dist);
    DBG(BLUE "<deproject> : " NONE "x : %.1f y : %.1f z : %.1f", point[0], point[1], point[2]);
    cv::Point3f p_w = camera2world(cv::Point3f(point[0], point[1], point[2]));
    return p_w;
}

void MyRealsense::calibrate(int n_row, int n_col)
{
    if (this->getState() == STOP) {DBG("please open device"); return;}
    this->n_rows = n_row;
    this->n_cols = n_col;
    this->setMode(COLOR_MODE);
    cv::Mat color_img;
    cv::GaussianBlur(this->color_img, color_img, cv::Size(3, 3), 0);
    cv::Mat gray_img;
    cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);
    this->pause();
    bool ret = cv::findChessboardCorners(gray_img, cv::Size(n_row, n_col), corners);
    if (corners.size()) {
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);
        cv::cornerSubPix(gray_img, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        cv::drawChessboardCorners(this->color_img, cv::Size(n_row, n_col), corners, ret);
        cv::circle(this->color_img, corners[0], 10, cv::Scalar(255, 0, 255), 3);
    }
    this->updateImg();
    DBG(BLUE "<cali> : " NONE "row : %d col : %d corner nums: %d", n_row, n_col, (int)corners.size());
    if ((int)corners.size() == n_row * n_col) {
        emit caliOK(true);
    } else {
        emit caliOK(false);
    }
}

std::vector<cv::Point3f> MyRealsense::getMualWorldCoor()
{
    std::vector<cv::Point3f> points;
    int co_x = is_x_inv ? -1 : 1;
    int co_y = is_y_inv ? -1 : 1;
    // int co_z = is_z_inv ? -1 : 1;

    for (int i = 0; i < n_cols; i++) {
        for (int j = 0; j < n_rows; j++) {
            if (cali_pos == UL)
                points.push_back(cv::Point3f(cali_x + j * cali_step * co_x, cali_y + i * cali_step * co_y, cali_z));
            else if (cali_pos == UR)
                points.push_back(cv::Point3f(cali_x - i * cali_step * co_x, cali_y + j * cali_step * co_y, cali_z));
            else if (cali_pos == DR)
                points.push_back(cv::Point3f(cali_x - j * cali_step * co_x, cali_y - i * cali_step * co_y, cali_z));
            else if (cali_pos == DL)
                points.push_back(cv::Point3f(cali_x + i * cali_step * co_x, cali_y - j * cali_step * co_y, cali_z));
        }
    }
    for (cv::Point3f point : points) {
        DBG(YELLOW "<point>" NONE "x : %f y : %f z : %f", point.x, point.y, point.z);
    }
    return points;
}

void MyRealsense::genExtrin()
{
    cv::Mat rvec;
    std::vector<cv::Point2f> img_points;
    std::vector<cv::Point3f> obj_points = getMualWorldCoor();
    cv::Mat intrinsic(cv::Matx33f(this->color_intrin.fx, 0, this->color_intrin.ppx,
                                  0, this->color_intrin.fy, this->color_intrin.ppy,
                                  0,                     0,                      1));
    cv::Mat distortion(cv::Matx14f(this->color_intrin.coeffs[0], this->color_intrin.coeffs[1], this->color_intrin.coeffs[2], this->depth_intrin.coeffs[3]));
    cv::solvePnPRansac(obj_points, corners, intrinsic, distortion, rvec, extrin_tmat);
    cv::Rodrigues(rvec, extrin_rmat);
    extrin_rmat.convertTo(extrin_rmat, CV_32F);
    extrin_tmat.convertTo(extrin_tmat, CV_32F);

    std::cout << "rmat" << extrin_rmat << std::endl;
    std::cout << "tmat" << extrin_tmat << std::endl;
}

cv::Point3f MyRealsense::camera2world(cv::Point3f point)
{
    cv::Mat p_c = cv::Mat(cv::Matx31f(point.x, point.y , point.z));
    cv::Mat p_w;
    cv::Mat rmat = extrin_rmat.t();
    try {
        p_w = rmat * (p_c - extrin_tmat);
    } catch (cv::Exception &e) {
        DBG(RED "<matrix error>" NONE);
        return cv::Point3f(0, 0, 0);
    }
    return cv::Point3f(p_w.at<float>(0), p_w.at<float>(1), p_w.at<float>(2));
}

void MyRealsense::run()
{
    DBG(GREEN "<run> : " NONE "thread start");

    device_list devs = ctx.query_devices();
    int n_dev = devs.size();
    DBG(BLUE "<device> : " NONE "device nums : %d", n_dev);

    if (n_dev) dev = devs[0];
    else {DBG(RED "<device> : " NONE "no device found"); return;}

    dep_sensor = new depth_sensor(dev.first<depth_sensor>());

    width = cover->width();
    height = cover->height();
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps_s);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps_s);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps_s);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps_s);

    pipeline_profile profile;
    try {
        profile = pipe.start(cfg);
    } catch (error &e) {
        DBG(RED "<pipe.start()> : %s" NONE, e.what());
        return;
    }
    DBG(GREEN "<camera> : " NONE " camera opened");
    state = WORK;

    // float depth_clipping_distance = 1.f;
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<video_stream_profile>();

    depth_intrin = depth_stream.get_intrinsics();
    color_intrin = color_stream.get_intrinsics();

    rs2::align align(RS2_STREAM_COLOR);
    while(this->getState() != STOP) {
        frameset frames = pipe.wait_for_frames(3000);
        frames = align.process(frames);
        color_frame = frames.get_color_frame();
        depth_frame = frames.get_depth_frame();

        if (!(color_frame && depth_frame)) {
            DBG(RED "<rs2 error> : " NONE "can not read");
            this->close(cover);
        }
        color_img = cv::Mat(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_img = cv::Mat(cv::Size(width, height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        emit frameOK();
    }
    DBG(GREEN "<run> : " NONE "thread stop");
}
}
