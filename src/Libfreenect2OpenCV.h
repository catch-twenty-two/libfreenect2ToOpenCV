#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

namespace libfreenect2opencv {

class Libfreenect2OpenCV {

    libfreenect2::PacketPipeline *m_pipeline;
    libfreenect2::Freenect2Device *m_dev;
    libfreenect2::Registration *m_registration;
    libfreenect2::SyncMultiFrameListener *m_listener;
    libfreenect2::Freenect2 m_freenect2;

    cv::Mat m_rgbMat;
    cv::Mat m_depthMat;
    cv::Mat m_depthMatUndistorted;
    cv::Mat m_IRMat;
    cv::Mat m_rgbdMat;
    cv::Mat m_rgbd2Mat;

    std::thread * m_updateThread;
    std::mutex m_updateMutex;
    std::condition_variable m_initSig;

    static void trampoline(Libfreenect2OpenCV * );

    void updateMat();

    enum Processor {
        cl, gl, cpu
    };

protected:
    static bool s_shutdown;

public:
    Libfreenect2OpenCV(Processor depthProcessor = Processor::gl);
    virtual ~Libfreenect2OpenCV();

    void start();
    void stop();

    const cv::Mat & getDepthMat()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_depthMat;
    }

    const cv::Mat & getDepthMatUndistorted()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_depthMatUndistorted;
    }

    const cv::Mat & getIRMat()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_IRMat;
    }

    const cv::Mat & getRGBd()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_rgbdMat;
    }

    const cv::Mat & getRGBd2()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_rgbd2Mat;
    }

    const cv::Mat & getRGBMat()
    {
        std::unique_lock<std::mutex> lock(m_updateMutex);
        return m_rgbMat;
    }
};

}
