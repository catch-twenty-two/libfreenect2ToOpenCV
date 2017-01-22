#include <iostream>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "Libfreenect2OpenCV.h"

using namespace std;

namespace libfreenect2opencv {

bool Libfreenect2OpenCV::s_shutdown = false;

Libfreenect2OpenCV::Libfreenect2OpenCV(Processor depthProcessor) :
        m_pipeline(nullptr),
        m_dev(nullptr),
        m_registration(nullptr),
        m_listener(nullptr),
        m_updateThread(nullptr)
{
    if (m_freenect2.enumerateDevices() == 0) {
        throw runtime_error("no device connected");
    }

    string serial = m_freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

    if (depthProcessor == Processor::cpu) {
        if (!m_pipeline) {
            m_pipeline = new libfreenect2::CpuPacketPipeline();
        }
    } else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if (!m_pipeline) {
            m_pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    } else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!m_pipeline) {
            m_pipeline = new libfreenect2::OpenCLPacketPipeline();
        }
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if (m_pipeline) {
        m_dev = m_freenect2.openDevice(serial, m_pipeline);
    } else {
        m_dev = m_freenect2.openDevice(serial);
    }

    if (m_dev == 0) {
        throw runtime_error("failure opening device");
    }

    m_listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                          libfreenect2::Frame::Depth |
                                                          libfreenect2::Frame::Ir);

    m_dev->setColorFrameListener(m_listener);
    m_dev->setIrAndDepthFrameListener(m_listener);

    m_dev->start();

    std::cout << "device serial: " << m_dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << m_dev->getFirmwareVersion() << std::endl;

    m_registration = new libfreenect2::Registration(m_dev->getIrCameraParams(),
                                                    m_dev->getColorCameraParams());
}

void Libfreenect2OpenCV::trampoline(Libfreenect2OpenCV * libfreenect2OpenCV)
{
    libfreenect2OpenCV->updateMat();
}

void Libfreenect2OpenCV::updateMat()
{
    libfreenect2::FrameMap frames;

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920,
                                                                                     1080 + 2,
                                                                                     4);
    while (!s_shutdown)
    {
        m_listener->waitForNewFrame(frames);

        std::unique_lock<std::mutex> lock(m_updateMutex);

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(m_rgbMat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(m_IRMat);

        m_IRMat /= 4096.0f;

        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(m_depthMat);

        m_depthMat /= 4096.0f;

        cv::resize(m_rgbMat, m_rgbMat, cv::Size(m_rgbMat.cols / 4, m_rgbMat.rows / 4));

        m_registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(m_depthMatUndistorted);

        m_depthMatUndistorted /= 4096.0f;

        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(m_rgbdMat);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(m_rgbd2Mat);
        cv::resize(m_rgbd2Mat, m_rgbd2Mat, cv::Size(m_rgbd2Mat.cols / 4, m_rgbd2Mat.rows / 4));

        m_listener->release(frames);

        m_initSig.notify_all();

        lock.unlock();
    }
}

void Libfreenect2OpenCV::stop()
{
    s_shutdown = true;

    m_updateThread->join();
}

void Libfreenect2OpenCV::start()
{
    m_updateThread = new std::thread( trampoline, this );

    std::unique_lock<std::mutex> lock(m_updateMutex);
    m_initSig.wait(lock);
}

Libfreenect2OpenCV::~Libfreenect2OpenCV()
{
    m_dev->stop();

    delete m_registration;
    delete m_updateThread;
}

}
