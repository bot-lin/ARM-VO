#include "ARM_VO.hpp"
#include <time.h>
#include <iostream>
#include "depthai/depthai.hpp"
static std::atomic<bool> withDepth{true};

static std::atomic<bool> outputDepth{false};
static std::atomic<bool> outputRectified{true};
static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    if (argc<3)
    {
        cerr << "Few input arguments!" << endl;
        cout << "Usage: ./exe pathToDataFolder paramsFile" << endl;
        return -1;
    }

    string imageDir = argv[1];
    string paramsFileName = argv[2];

    ARM_VO VO(paramsFileName);
    Viewer Results;

    unsigned int FPS, sum_fps = 0;
    unsigned int imageCounter = 0;

    char imageName[100];
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDisp = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    if(withDepth) {
        // StereoDepth
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        // stereo->setInputResolution(1280, 720);
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Linking
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        stereo->disparity.link(xoutDisp->input);

        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }

        if(outputDepth) {
            stereo->depth.link(xoutDepth->input);
        }

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 8, false);
    auto rightQueue = device.getOutputQueue("right", 8, false);
    auto dispQueue = withDepth ? device.getOutputQueue("disparity", 8, false) : nullptr;
    auto depthQueue = withDepth ? device.getOutputQueue("depth", 8, false) : nullptr;
    auto rectifLeftQueue = withDepth ? device.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? device.getOutputQueue("rectified_right", 8, false) : nullptr;

    // Disparity range is used for normalization
    float disparityMultiplier = withDepth ? 255 / stereo->initialConfig.getMaxDisparity() : 0;

    while (true)
    {
        auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();

        Mat curr_frame = rectifL->getFrame(); //Load as grayscale

        if (curr_frame.empty())
        {
            cerr << "Can't read the image " << endl;
            break;
        }

        if (!VO.initialized)
        {
            VO.init(curr_frame);
        }
        else
        {
            clock_t start = clock();

            VO.update(curr_frame);

            clock_t finish = clock();
            FPS = 1000 / (1000*(finish-start)/CLOCKS_PER_SEC);
            sum_fps+=FPS;

            Results.show(curr_frame, VO.prev_inliers, VO.curr_inliers, FPS, VO.t_f);
        }

        imageCounter++;
    }

    cout << "Finished" << endl;
    cout << "Average FPS: " << setprecision(2) << 1.0f*sum_fps/imageCounter << endl;

    destroyAllWindows();

    return 1;
}
