#pragma once

#include <condition_variable>
#include <mutex>

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
    libfreenect2::Freenect2 freenect2;

    cv::Mat m_rgbMat;
    cv::Mat m_depthMat;
    cv::Mat m_depthMatUndistorted;
    cv::Mat m_irMat;
    cv::Mat m_rgbdMat;
    cv::Mat m_rgbd2Mat;

    std::mutex updateMutex;
    std::mutex initMutex;

    std::condition_variable initSig; // condition variable for critical section

    static void trampoline(Libfreenect2OpenCV * );

    void updateMat();

    enum Processor {
        cl, gl, cpu
    };

protected:
    static bool s_shutdown;    // Whether the running application should shut down.

public:
    Libfreenect2OpenCV();
    virtual ~Libfreenect2OpenCV();

    void start();
    void stop();

    void waitBuf() {
        std::unique_lock<std::mutex> lock(initMutex);
        initSig.wait(lock);
    }

    const cv::Mat & getDepthMat()
    {
        waitBuf();
        return m_depthMat;
    }

    const cv::Mat & getDepthMatUndistorted()
    {
        waitBuf();
        return m_depthMatUndistorted;
    }

    const cv::Mat & getIrMat()
    {
        waitBuf();
        return m_irMat;
    }

    const cv::Mat & getRgbd()
    {
        waitBuf();
        return m_rgbdMat;
    }

    const cv::Mat & getRgbd2()
    {
        waitBuf();
        return m_rgbd2Mat;
    }

    const cv::Mat & getRgbMat()
    {
        waitBuf();
        return m_rgbMat;
    }

};

} /* namespace Communicator */
