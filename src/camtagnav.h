#pragma once

/* --------------------------------------------------------------------------
    Header
   -------------------------------------------------------------------------- */

// Save the marker in a std::map. Marker id -> marker_t entries.
#include <map>
#include <list>
#include <string>
#include <vector>

// Eigen Math lib
#include <Eigen/Dense>

// April tags detector and various families that can be selected by command line option
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"

// OpenCV
#include "opencv2/opencv.hpp"

/* --------------------------------------------------------------------------
    Defines
   -------------------------------------------------------------------------- */

#define CAMTAGNAV_CONFIG_FILE "camtagconfig.xml"

/* --------------------------------------------------------------------------
    Structs
   -------------------------------------------------------------------------- */

// AprilTag entry from marker.txt
struct marker_t
{
    int id; // AprilTag id

    // Four corners of the AprilTag
    // lower left, lower right, upper right, upper left
    Eigen::Vector3d corners[4]; // world coordinates

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct position_t
{
    double xyz[3];
};

/* --------------------------------------------------------------------------
    Marker Database Class
   -------------------------------------------------------------------------- */

/**
 * Store 4x 3D local coordinates for each marker.
 */
class CMarkerDB
{
public:
    // The marker database is a simple std::map
    // Basically it would be std::map<int, marker_t> m_marker,
    // but corners[4] is a Eigen::Vector3d, we need to do this
    // aligned_allocator stuff to keep Eigen happy.
    std::map<int, marker_t, std::less<int>, Eigen::aligned_allocator<std::pair<const int, marker_t> > > m_marker;

    // Load the marker database.
    // The database has one entry in each line:
    //  id  corner_0_x corner_0_y corner_0_z corner_1_x corner_1_y corner_1_z corner_2_x corner_2_y corner_2_z corner_3_x corner_3_y corner_3_z 
    // Returns true if everything is ok.
    bool LoadFromFile(const char* path);
};

/* --------------------------------------------------------------------------
    Core Logic for PnP estimation based on AprilTags
   -------------------------------------------------------------------------- */

class CamTagNavApp
{
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?

    std::vector<std::string> m_imgNames; // for post processing. list of strings with image files

    // Manual exposure settings (Linux/V4L only)
    int m_exposure;
    int m_gain;
    int m_brightness;

    int m_deviceId; // OpenCV camera id (in case of multiple cameras)

    bool m_detectionActive;
    cv::Mat m_cameraMatrix; // open cv style camera matrix
    cv::Mat m_distCoeffs; // open cv style distortion coefficients
    CMarkerDB m_marker_db;
    bool m_robust; // robust adjustment with solvePnPRansac
    bool m_solveEPNP;
    double m_robustAlpha;
    int m_robustMaxIter;
    double m_robustMaxReprojError;
    int m_ransacCornerPoints; // how many corner points are used for solvePnPRansac
    double m_scaleWidth;
    double m_scaleHeight;
    cv::Mat m_prevR; // previous rotation for solvePnP
    cv::Mat m_prevT; // previous translation for solvePnP
    bool m_useGuess; // use previous position for solvePnP
    double m_largestAcceptableResidual; // don't accept a solution if there are super high residuals (in pixel)
    int m_minMarkerCount;
    float m_minMarkerArea;
    bool m_showUndist;
    double m_pixelDetectionPrecision; // for covariance estimation: how accurate can we detect pixels (one sigma). faster calculation if set to 1.0
    double m_minPositionSigma; // if a solution sigma is larger than this, reject the solution
    bool m_showResiduals; // plot 4 lines where the marker corner was expected to be in the image

protected:
    /* Helper function for estimatePose to calculate the residuals in
    image space. If Qxx is not NULL, the covariance
    matrix for the position and orientation is calculated.
    Qxx layout: 3 rvec, 3 tvec */
    void calculateResiduals(const std::vector<cv::Point3f>& obj_pts,
        const std::vector<cv::Point2f>& img_pts,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        const cv::Mat& c_r_w, const cv::Mat& c_t_w,
        std::vector<cv::Point2f>& residuals,
        Eigen::Matrix<double, 6, 6>* Qxx = NULL) const;

    /* Helper function for estimatePose(...), this function
       performs the bundle adjustment to solve the PnP problem.
       The Qxx matrix has the rodriguez vector and the translation
       as parameters. So this is not directly the quaternion or r/p/y error */
    bool estimatePoseCore(vector<AprilTags::TagDetection>& detections,
        Eigen::Matrix<double, 6, 6>& Qxx,
        bool robust,
        bool use_guess,
        int solvePnPflags,
        cv::Mat& c_r_w,
        cv::Mat& c_t_w) const;

    /* estimate position and orientation of the camera based on the
    image space observations by the AprilTags tag detector */
    bool estimatePose(vector<AprilTags::TagDetection>& detections,
        Eigen::Vector3d& pos,
        Eigen::Quaterniond& q,
        Eigen::Matrix<double, 6, 6>& Qxx);

public:

    // default constructor
    CamTagNavApp() :
        m_tagDetector(NULL),
        m_tagCodes(AprilTags::tagCodes36h11),

        m_draw(true),

        m_exposure(-1),
        m_gain(-1),
        m_brightness(-1),
        m_deviceId(0),
        m_robust(false),
        m_solveEPNP(false),
        m_robustAlpha(0),
        m_robustMaxIter(0),
        m_robustMaxReprojError(0.0),
        m_useGuess(false),
        m_largestAcceptableResidual(0.0),
        m_ransacCornerPoints(4),
        m_scaleWidth(1.0),
        m_scaleHeight(1.0),
        m_minMarkerCount(1),
        m_minMarkerArea(2000.0f),
        m_showUndist(false),
        m_minPositionSigma(1.0),
        m_pixelDetectionPrecision(1.0),
        m_detectionActive(true),
        m_showResiduals(true)
    {
        printf("OpenCV Version %i.%i\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);

        m_prevR = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));
        m_prevT = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));

        m_cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        m_distCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    }

    ~CamTagNavApp()
    {
    }

    // changing the tag family
    void setTagCodes(std::string s);

    // parse command line options to change default behavior
    void parseOptions(cv::FileStorage& fs_config);

    void setup();

    bool loadMarkerDB();

    void setupVideo(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    void toggleUndist() { m_showUndist = !m_showUndist;  }

    cv::Mat processImage(cv::Mat image);

    /** Load and process a single image */
    void setImages(const std::vector<std::string> imgNames) { m_imgNames = imgNames; }
    /** Load images specified by setImage(...) */
    void loadImages();

    // Video or image processing?
    bool isVideo();

    void toggleMarkerDetection() { m_detectionActive = !m_detectionActive; }
};
