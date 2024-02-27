/*
 * Written by Jan Zwiener, 2024 (jan@zwiener.org)
 */

/* --------------------------------------------------------------------------
    Header
   -------------------------------------------------------------------------- */

#include <stdio.h>
#include <iostream>

#include "camtagnav.h"

/* --------------------------------------------------------------------------
    Defines
   -------------------------------------------------------------------------- */

#define CALIBRATION_FILE_PATH  "camera.xml" /**< default path to calib. file */

/* --------------------------------------------------------------------------
    Local Functions
   -------------------------------------------------------------------------- */

static bool init(int argc, char** argv,
                 cv::VideoCapture& cap, CamTagNavApp& app);
static bool setupVideo(int deviceId, cv::VideoCapture& cap,
                       int& width, int& height,
                       bool openPropDialog);
static bool xmlReadStringList(const string& filename, vector<string>& l);
static void loop(cv::VideoCapture& cap,
                 CamTagNavApp& app);

/* --------------------------------------------------------------------------
    Functions
   -------------------------------------------------------------------------- */

int main(int argc, char* argv[])
{
#ifdef _WIN32
    // Required on Windows to send data over UDP sockets
    WSAData data;
    WSAStartup(MAKEWORD(2, 2), &data);
#endif

    CamTagNavApp app;
    cv::VideoCapture cap;

    if (!init(argc, argv, cap, app))
    {
        std::cout << "Init failed" << std::endl;
        return -1;
    }

    if (app.isVideo())
    {
        loop(cap, app);
    }
    else
    {
        app.loadImages();
    }

    return 0;
}

static bool init(int argc, char** argv,
                 cv::VideoCapture& cap,
                 CamTagNavApp& app)
{
    const int def_width = 640;
    const int def_height = 480;
    int width, height;
    std::string calibration_file_path;

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    cv::Mat distCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

    // some (random) default camera attributes
    cameraMatrix.at<double>(0, 0) = 600;
    cameraMatrix.at<double>(1, 1) = 600;
    cameraMatrix.at<double>(0, 2) = def_width/2;
    cameraMatrix.at<double>(1, 2) = def_height/2;
    cameraMatrix.at<double>(2, 2) = 1.0;

    cv::FileStorage fs_config;
    try
    {
        fs_config.open(CAMTAGNAV_CONFIG_FILE, cv::FileStorage::READ);
    }
    catch (...)
    {
        cerr << "Failed to configuration file" << std::endl;
    }

    int deviceId;
    int openPropDialog = 0;
    if (fs_config.isOpened())
    {
        fs_config["show_cam_properties"] >> openPropDialog;
        fs_config["width"] >> width;
        fs_config["height"] >> height;
        fs_config["device_id"] >> deviceId;
        if (width == 0)
            width = def_width;
        if (height == 0)
            height = def_height;

        // check if the user wants to process a list of files (e.g. pngs)
        // image_list points to a .xml file with file paths:
        // e.g. image_list = "input.xml",
        // xmlReadStringList will read "input.xml"
        //
        /*
            "<?xml version=\"1.0\"?>\n"
            "<opencv_storage>\n"
            "<images>\n"
            "view000.png\n"
            "view001.png\n"
            "<!-- view002.png -->\n"
            "view003.png\n"
            "view010.png\n"
            "one_extra_view.jpg\n"
            "</images>\n"
            "</opencv_storage>\n";
        */
        std::string image_list_xml_path;
        fs_config["image_list"] >> image_list_xml_path;
        if (image_list_xml_path.length() > 1)
        {
            std::vector<std::string> list;
            xmlReadStringList(image_list_xml_path, list);
            if (list.size() > 0)
                app.setImages(list);
        }

        fs_config["calibration"] >> calibration_file_path;
        if (calibration_file_path.length() < 1)
        {
            calibration_file_path = CALIBRATION_FILE_PATH; // use default
        }
    }
    else
    {
        cerr << "Could not open config file: " << CAMTAGNAV_CONFIG_FILE << std::endl;
        return false;
    }

    std::cout << "Calibration file path: " << calibration_file_path << std::endl;

    int calib_width, calib_height;
    cv::FileStorage fs;
    try {
        fs.open(calibration_file_path, cv::FileStorage::READ);
    }
    catch (...)
    { }

    if (!fs.isOpened())
    {
        cerr << "Could not open camera calibration file: " << calibration_file_path << std::endl;
        return false;
    }
    else
    {
        try {
            fs["camera_matrix"] >> cameraMatrix;
            fs["distortion_coefficients"] >> distCoeffs;
            std::cout << "camera matrix: " << std::endl << cameraMatrix << std::endl;
            std::cout << "dist coeffs: " << std::endl << distCoeffs.t() << std::endl;
            fs["image_width"] >> calib_width;
            fs["image_height"] >> calib_height;

            std::string calib_time;
            fs["calibration_time"] >> calib_time;
            if (calib_time.length() > 0)
                std::cout << "Calibration date: " << calib_time << endl;
            else
                std::cout << "Calibration date unknown." << endl;

        }
        catch (cv::Exception e)
        {
            std::cout << "Calibration file (" << calibration_file_path << ") is invalid" << std::endl;
            return false;
        }

        if (calib_width == 0 || calib_height == 0)
        {
            std::cout << "Invalid config file. Calibration width and height "
                         "(image_width, image_height) is missing." << std::endl;
            return false;
        }
    }

    if (!app.loadMarkerDB())
    {
        std::cout << "Could not load marker database" << std::endl;
        return false;
    }

    if (app.isVideo())
    {
        if (!setupVideo(deviceId, cap, width, height, openPropDialog != 0))
        {
            std::cout << "Video input error" << std::endl;
            exit(-1);
        }
    }

    // Scale cameraMatrix for actual resolution
    const double scale_x = width / calib_width;
    const double scale_y = height / calib_height;
    cameraMatrix.at<double>(0, 0) *= scale_x;
    cameraMatrix.at<double>(1, 1) *= scale_y;
    cameraMatrix.at<double>(0, 2) *= scale_x;
    cameraMatrix.at<double>(1, 2) *= scale_y;

    app.parseOptions(fs_config);
    app.setup();
    app.setupVideo(cameraMatrix, distCoeffs);

    return true;
}

static bool setupVideo(int deviceId, cv::VideoCapture& cap,
                       int& width, int& height,
                       bool openPropDialog)
{
    std::cout << "Opening camera id " << deviceId << "..." << std::endl;
    try
    {
        cap = cv::VideoCapture(deviceId);
    }
    catch (...) {}
    if (!cap.isOpened())
    {
        cerr << "ERROR: Can't open video device " << deviceId << "\n";
        return false;
    }
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, width))
        std::cout << "Failed to set frame width to " << width << endl;
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, height))
        std::cout << "Failed to set frame height to " << height << endl;

    double new_width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double new_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "Actual resolution: "
         << new_width << "x" << new_height << endl;
    width = (int)new_width;
    height = (int)new_height;

    if (openPropDialog)
    {
        cap.set(cv::CAP_PROP_SETTINGS, 1);
    }

    return true;
}

static void loop(cv::VideoCapture& cap,
                 CamTagNavApp& app)
{
    cv::Mat image_orig;

    int frame = 0;

    while (true) /* main loop until ESC is pressed */
    {
        bool gotImg = false;
        if (cap.grab())
        {
            gotImg = cap.retrieve(image_orig);
            if (gotImg)
            {
                app.processImage(image_orig.clone());
            }
        }

        frame++;
        int key = cv::waitKey(gotImg ? 1 : 5);
        if (key == 27) // exit if ESC key is pressed
        {
            break;
        }
        if (key == 'u')
        {
            app.toggleUndist();
        }
        if (key == 'p')
        {
            cap.set(cv::CAP_PROP_SETTINGS, 1);
        }
        if (key == 'd')
        {
            app.toggleMarkerDetection();
        }
    }
}

/** Read a list of file names from a OpenCV storage.
 * @param[in] filename Path to xml file
 * @param[out] l Vector to fill with strings from xml file.
 *
 * Example .xml file, l is filled with "view000.png", "view001.png", ...
 *
 *          "<?xml version=\"1.0\"?>\n"
 *          "<opencv_storage>\n"
 *          "<images>\n"
 *          "view000.png\n"
 *          "view001.png\n"
 *          "<!-- view002.png -->\n"
 *          "view003.png\n"
 *          "view010.png\n"
 *          "one_extra_view.jpg\n"
 *          "</images>\n"
 *          "</opencv_storage>\n";
 * */
static bool xmlReadStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}
