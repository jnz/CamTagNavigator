/*
 * Written by Jan Zwiener, 2024 (jan@zwiener.org)
 */

#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "camtagnav.h"
#include "cv2eigen.h"

/* --------------------------------------------------------------------------
    Defines
   -------------------------------------------------------------------------- */

static const char* WINDOWNAME = "CamTagNavigator";

/* --------------------------------------------------------------------------
    CMarkerDB (class to load the marker id and corner positions)
   -------------------------------------------------------------------------- */

bool CMarkerDB::LoadFromFile(const char* path)
{
    FILE* f = fopen(path, "r");
    if (!f)
    {
        printf("Could not open file: %s\n", path);
        return false;
    }
    char linebuf[1024];
    int id;
    double corner[4][3];
    int i, j;
    int line = 0;

    while (!feof(f))
    {
        line++;
        if (!fgets(linebuf, sizeof(linebuf), f))
            continue;

        // lower left xyz, lower right xyz, upper right xyz, upper left xyz
        const int items =
            sscanf(linebuf, "%i "
                "%lf %lf %lf "
                "%lf %lf %lf "
                "%lf %lf %lf "
                "%lf %lf %lf ",
                &id,
                &corner[0][0], &corner[0][1], &corner[0][2],
                &corner[1][0], &corner[1][1], &corner[1][2],
                &corner[2][0], &corner[2][1], &corner[2][2],
                &corner[3][0], &corner[3][1], &corner[3][2]);

        if (items > 0 && items != 13)
        {
            printf("Err on line %i, items %i expected 13: %s\n", line, items, linebuf);
            continue;
        }

        marker_t m;
        m.id = id;
        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 3; j++)
            {
                m.corners[i](j) = corner[i][j];
            }
        }

        if (m_marker.find(id) != m_marker.end())
            printf("Warning duplicated marker id in file: %i\n", id);

        m_marker[id] = m;
    }

    if (m_marker.size() == 0)
        printf("No markers found in file\n");
    return m_marker.size() > 0;
}

/* --------------------------------------------------------------------------
    CamTagNavApp (contains the core app logic)
   -------------------------------------------------------------------------- */

void CamTagNavApp::setTagCodes(std::string s)
{
    if (s == "16h5") {
        m_tagCodes = AprilTags::tagCodes16h5;
    }
    else if (s == "25h7") {
        m_tagCodes = AprilTags::tagCodes25h7;
    }
    else if (s == "25h9") {
        m_tagCodes = AprilTags::tagCodes25h9;
    }
    else if (s == "36h9") {
        m_tagCodes = AprilTags::tagCodes36h9;
    }
    else if (s == "36h11") {
        m_tagCodes = AprilTags::tagCodes36h11;
    }
    else {
        std::cout << "Invalid tag family specified" << endl;
        exit(1);
    }
}

// parse command line options to change default behavior
void CamTagNavApp::parseOptions(cv::FileStorage& fs_config)
{
    if (!fs_config.isOpened())
        return;

    int robust;
    int epnp;
    int useguess;
    int min_marker_count;
    int robust_corner_points;
    float min_marker_area;
    int show_undist;
    double minPositionSigma;
    double pixelDetectionPrecision;
    int showResiduals;

    try {
        fs_config["robust"] >> robust;
        fs_config["robust_alpha"] >> m_robustAlpha;
        fs_config["robust_max_iter"] >> m_robustMaxIter;
        fs_config["robust_max_reproj_error"] >> m_robustMaxReprojError;
        fs_config["robust_corner_points"] >> robust_corner_points;
        fs_config["epnp"] >> epnp;
        fs_config["use_guess"] >> useguess;
        fs_config["max_reproj_error"] >> m_largestAcceptableResidual;
        fs_config["min_marker_count"] >> min_marker_count;
        fs_config["min_marker_area"] >> min_marker_area;
        fs_config["show_undist"] >> show_undist;
        fs_config["min_position_sigma"] >> minPositionSigma;
        fs_config["show_residuals"] >> showResiduals;
        fs_config["udp_port"] >> m_udpPort;
        fs_config["udp_host"] >> m_udpTargetHost;

        std::string tag_code;
        fs_config["tag_code"] >> tag_code;
        if (tag_code.length() > 0)
        {
            std::cout << "Setting tag code:" << tag_code << std::endl;
            setTagCodes(tag_code);
        }
        if (epnp > 0)
        {
            std::cout << "EPNP mode active" << std::endl;
            m_solveEPNP = true;
        }
        if (m_largestAcceptableResidual < 1.0)
        {
            m_largestAcceptableResidual = 8.0;
        }
        m_showResiduals = (showResiduals != 0);
    }
    catch (cv::Exception e) {
    }

    m_useGuess = (useguess != 0);
    m_robust = (robust != 0);
    if (m_robustAlpha == 0.0)
        m_robustAlpha = 0.05;
    if (m_robustMaxIter == 0.0)
        m_robustMaxIter = 500;
    if (m_robustMaxReprojError == 0.0)
        m_robustMaxReprojError = 4.0;
    if (min_marker_count > 0)
        m_minMarkerCount = min_marker_count;
    if (min_marker_area > 100.0f)
        m_minMarkerArea = min_marker_area;
    if (minPositionSigma > 0.0001)
        m_minPositionSigma = minPositionSigma;
    if (pixelDetectionPrecision > 0.00001)
        m_pixelDetectionPrecision = pixelDetectionPrecision;

    m_showUndist = (show_undist > 0);
    if (robust_corner_points > 0 && robust_corner_points <= 4)
    {
        m_ransacCornerPoints = robust_corner_points;
    }
}

void CamTagNavApp::setup(double avg_reprojection_error)
{
    m_pixelDetectionPrecision = avg_reprojection_error;
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw)
    {
        cv::namedWindow(WINDOWNAME, 1);
    }

    if (m_robust)
    {
        std::cout << "Robust mode ENABLED" << std::endl;
        std::cout << "Alpha: " << m_robustAlpha*100.0 << "% (1.0 - alpha = " << (1.0 - m_robustAlpha) << ")" << std::endl;
        std::cout << "Max. reproj error: " << m_robustMaxReprojError << " px" << std::endl;
    }
    else
    {
        std::cout << "Robust mode DISABLED" << std::endl;
    }
    if (m_useGuess)
    {
        std::cout << "Use previous position for extrinsic guess" << std::endl;
    }
    std::cout << "Max. acceptable position sigma: " << m_minPositionSigma << " m" << std::endl;
    std::cout << "Max. acceptable reprojection error: " << m_largestAcceptableResidual << " px" << std::endl;
    std::cout << "Minimum valid marker count: " << m_minMarkerCount << std::endl;
    std::cout << "Minimum visible marker area: " << m_minMarkerArea << " px*px" << std::endl;
    std::cout << "Robust corner points: " << m_ransacCornerPoints << std::endl;
    std::cout << "Pixel detection precision: " << m_pixelDetectionPrecision << " px" << std::endl;

    std::cout << "Sending output to: " << m_udpTargetHost << ":" << m_udpPort << std::endl;
    m_udpsender.SetRemoteAddr(m_udpTargetHost.c_str(), m_udpPort);
}

bool CamTagNavApp::loadMarkerDB()
{
    const char* pathmarker = "marker.txt";
    if (!m_marker_db.LoadFromFile(pathmarker))
    {
        printf("Error: no marker database found [%s]\n", pathmarker);
        return false;
    }
    return true;
}

void CamTagNavApp::setupVideo(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    m_cameraMatrix = cameraMatrix;
    m_distCoeffs = distCoeffs;
}

void CamTagNavApp::calculateResiduals(const std::vector<cv::Point3f>& obj_pts,
                        const std::vector<cv::Point2f>& img_pts,
                        const cv::Mat& cameraMatrix,
                        const cv::Mat& distCoeffs,
                        const cv::Mat& c_r_w, const cv::Mat& c_t_w,
                        std::vector<cv::Point2f>& residuals,
                        Eigen::Matrix<double, 6, 6>* Qxx) const
{
    if (obj_pts.size() != img_pts.size())
        return; // this should not happen

    float u, v;
    std::vector<cv::Point2f> projPoints;
    cv::Mat cvJacobian;
    const bool calcQxx = (Qxx != NULL);

    cv::projectPoints(obj_pts,
        c_r_w, c_t_w,
        cameraMatrix, distCoeffs,
        projPoints,
        calcQxx ? cvJacobian : cv::noArray());
    for (int i = 0; i < (int)projPoints.size(); i++)
    {
        u = img_pts[i].x - projPoints[i].x;
        v = img_pts[i].y - projPoints[i].y;
        residuals.push_back(cv::Point2f(u, v));
    }

    if (!calcQxx)
        return;

    // OK, let's calculate the covariance matrix Qxx
    // 6x6 matrix
    // rvec (3) tvec(3)

    // Jacobian layout from projectPoints():
    // (from OpenCV documentation 3.1)
    // 2*N rows
    // 10 + ndistCoeffs cols:
    //   - 3 rvec
    //   - 3 tvec
    //   - 2 focal
    //   - 2 principal point

    // The goal here is to calculate the covariance matrix
    // for the PnP problem.
    // projectPoints() returns a full jacobi matrix
    // but we only care about orientation and translation.
    // (if not, adjust this code)

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> jacobianLarge; // complete jacobian
    Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian; // only rvec and tvec
    cv2eigen(cvJacobian, jacobianLarge);
    const int rows = jacobianLarge.rows();
    jacobian = jacobianLarge.block(0, 0, rows, 6);

    Eigen::MatrixXd Qxxinv;
    // for a m_pixelDetectionPrecision of 1, we don't have to create
    // the identity weight matrix
    if (m_pixelDetectionPrecision != 1.0)
    {
        Eigen::MatrixXd P;
        P.resize(rows, rows);
        P.setZero();
        const double w = 1.0 / (m_pixelDetectionPrecision*m_pixelDetectionPrecision);
        P.diagonal().setConstant(w);
        Qxxinv = jacobian.transpose()*P*jacobian;
    }
    else
    {
        Qxxinv = jacobian.transpose()*jacobian;
    }

    // finally calculate the Qxx matrix.
    *Qxx = Qxxinv.inverse();
}

/* Helper functio for estimatePose(...) */
bool CamTagNavApp::estimatePoseCore(vector<AprilTags::TagDetection>& detections,
      Eigen::Matrix<double, 6, 6>& Qxx,
      bool robust,
      bool use_guess,
      int solvePnPflags,
      cv::Mat& c_r_w,
      cv::Mat& c_t_w) const
{
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point3f> obj_pts;
    // not every detection is used in the adjustment
    // save link between adjustment indices to detection ids
    std::vector<int> point_index_to_detection;
    // the idea here is to only use n corner points for the ransac
    // estimation.
    const int corner_points = robust ? m_ransacCornerPoints : 4;

    // if we got enough markers, check if the covered area is large
    // enough to estimate a position
    {
        float area = 0.0f;
        int good_markers = 0;
        for (int i = 0; i < (int)detections.size(); i++) {
            if (!detections[i].good)
               continue;
            if (detections[i].outlier)
                continue;
            area += detections[i].pixelArea();
            good_markers++;
        }
        if ((good_markers < m_minMarkerCount) && (area < m_minMarkerArea))
        {
            printf("Not enough marker. Count %i/%i, area %.1f/%.1f\n",
                good_markers, m_minMarkerCount, area, m_minMarkerArea);
            return false;
        }
        if (!robust)
            printf("A%5.0f ", area);
    }

    Eigen::Vector3d point3d_opencv;
    for (int i = 0; i < (int)detections.size(); i++)
    {
        const int id = detections[i].id;

        if (!detections[i].good)
            continue;
        if (detections[i].outlier)
            continue;

        // Without 'auto' this would look like this:
        // const std::map<int, marker_t, std::less<int>,
        //  Eigen::aligned_allocator<std::pair<const int, marker_t> >
        //  >::const_iterator iter =
        const auto iter =
            m_marker_db.m_marker.find(id);
        if (iter == m_marker_db.m_marker.end())
            continue;

        const marker_t& m = iter->second;

        for (int j = 0; j < corner_points; j++)
        {
            img_pts.push_back(cv::Point2d(detections[i].p[j].first,
                detections[i].p[j].second));

            // transform into OpenCV coordinate system
            point3d_opencv = m.corners[j];
            obj_pts.push_back(cv::Point3d(point3d_opencv.x(),
                                          point3d_opencv.y(),
                                          point3d_opencv.z()));

            point_index_to_detection.push_back(i);
        }
    }
    if (obj_pts.empty())
    {
        printf("No valid measurements.\n");
        return false;
    }
    if (obj_pts.size() != img_pts.size())
    {
        printf("3D/2D measurement count wrong. Data error? ");
        return false;
    }

    bool result;
    std::vector<int> inliers;
    try {

        if (robust)
        {
            result =
                cv::solvePnPRansac(obj_pts,
                    img_pts,
                    m_cameraMatrix,
                    m_distCoeffs,
                    c_r_w,
                    c_t_w,
                    use_guess,
                    m_robustMaxIter,
                    (float)m_robustMaxReprojError,
                    (1.0 - m_robustAlpha),
                    inliers, solvePnPflags);

            for (int j = 0; j < (int)detections.size(); j++)
            {
                detections[j].outlier = true;
            }
            for (int j = 0; j < (int)inliers.size(); j++)
            {
                detections[point_index_to_detection[j]].outlier = false;
            }
        }
        else
        {
            result = cv::solvePnP(obj_pts, img_pts, m_cameraMatrix, m_distCoeffs, c_r_w, c_t_w, use_guess, solvePnPflags);
        }
    }
    catch (...)
    {
        printf("Bundle adjustment failed. ");
        result = false;
    }

    std::vector<cv::Point2f> residuals;
    calculateResiduals(obj_pts, img_pts, m_cameraMatrix, m_distCoeffs, c_r_w, c_t_w, residuals, &Qxx);
    int i = 0;
    double largest_residual = 0;
    for (int j = 0; j < (int)residuals.size(); j++)
    {
        detections[point_index_to_detection[j]].residuals[i].first = residuals[j].x;
        detections[point_index_to_detection[j]].residuals[i].second = residuals[j].y;

        i++;
        if (i >= corner_points)
            i = 0;

        if (detections[point_index_to_detection[j]].outlier)
            continue;

        const float du = fabsf(residuals[j].x);
        const float dv = fabsf(residuals[j].y);
        if (du > largest_residual)
            largest_residual = du;
        if (dv > largest_residual)
            largest_residual = dv;
    }
    if (result)
    {
        const float sigmaAbs = sqrtf((float)Qxx(3, 3) +
                                     (float)Qxx(4, 4) +
                                     (float)Qxx(5, 5));
        if (sigmaAbs > (float)m_minPositionSigma)
        {
            const float sigmaX = sqrtf((float)Qxx(3, 3));
            const float sigmaY = sqrtf((float)Qxx(4, 4));
            const float sigmaZ = sqrtf((float)Qxx(5, 5));
            printf("Solution accuracy bad: (%.1f %.1f %.1f) %.1f > %.1f\n",
                sigmaX, sigmaY, sigmaZ, sigmaAbs, (float)m_minPositionSigma);
            result = false;
        }
    }

    if (result == false)
    {
        printf("solvePnP failed (%s).", robust ? "robust" : "L2");
        return false;
    }

    if (!robust)
    {
        if (largest_residual > m_largestAcceptableResidual)
        {
            printf("Large residual: %.1f px. ", (float)largest_residual);
            result = false;
        }
    }

    if (!result)
    {
        return false;
    }

    return true;
}

bool CamTagNavApp::estimatePose(
    vector<AprilTags::TagDetection>& detections,
    Eigen::Vector3d& pos,
    Eigen::Quaterniond& q,
    Eigen::Matrix<double, 6, 6>& Qxx)
{
    cv::Mat c_r_w, c_t_w, c_R_w;
    c_r_w = m_prevR;
    c_t_w = m_prevT;
    const bool use_guess = (m_useGuess &&
        m_prevT.at<double>(0, 0) != 0.0 &&
        m_prevT.at<double>(1, 0) != 0.0 &&
        m_prevT.at<double>(2, 0) != 0.0);

    int solvePnPflags = 0;
    if (m_solveEPNP)
        solvePnPflags |= cv::SOLVEPNP_EPNP;
    else
        solvePnPflags |= cv::SOLVEPNP_ITERATIVE;

    bool result = estimatePoseCore(detections,
        Qxx,
        m_robust,
        use_guess,
        solvePnPflags,
        c_r_w,
        c_t_w);
    if (result && m_robust)
    {
        result = estimatePoseCore(detections,
            Qxx,
            false,
            use_guess,
            solvePnPflags,
            c_r_w,
            c_t_w);
    }
    if (!result)
        return false;

    m_prevR = c_r_w;
    m_prevT = c_t_w;

    // c_r_w = rotation from world to camera
    cv::Rodrigues(c_r_w, c_R_w);

    Eigen::MatrixXd R_opencv_to_cam;
    cv2eigen(c_R_w, R_opencv_to_cam);
    Eigen::Matrix3d R_cam_to_opencv = R_opencv_to_cam.transpose();
    q = R_cam_to_opencv;

    cv::Mat c_T_w(c_t_w);
    cv::Mat w_R_c(c_R_w.t());
    cv::Mat w_T_c = -w_R_c * c_T_w;
    double *pt = w_T_c.ptr<double>();
    pos = Eigen::Vector3d(pt[0], pt[1], pt[2]);

    return true;
}

cv::Mat CamTagNavApp::processImage(cv::Mat image)
{
    cv::Mat image_gray;

    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    vector<AprilTags::TagDetection> detections;
    detections = m_tagDetector->extractTags(image_gray);

    Eigen::Vector3d camT; // cam pos in local system
    Eigen::Quaterniond q; // camera to local
    Eigen::Matrix<double, 6, 6> Qxx;
    camT.setZero();
    q.setIdentity();

    if (m_detectionActive && estimatePose(detections, camT, q, Qxx))
    {
        printf("POS %7.3f %7.3f %7.3f ",
            camT(0), camT(1), camT(2));

        // Estimated precision:
        printf("(%5.3f %5.3f %5.3f) ",
            sqrt(Qxx(0, 0)), sqrt(Qxx(1, 1)), sqrt(Qxx(2, 2)));

        printf("\n");

        emitSolution(camT, q);

        // double roll = atan2(R_cam_to_ned(2, 1), R_cam_to_ned(2, 2));
        // double pitch = asin(-R_cam_to_ned(2, 0));
        // double yaw = atan2(R_cam_to_ned(1, 0), R_cam_to_ned(0, 0));
        // printf("RPY %6.1f %6.1f %6.1f ", roll*180.0 / M_PI, pitch*180.0 / M_PI, yaw*180.0 / M_PI);
    }

    // show the current image including any detections
    if (m_draw)
    {
        // cv::cvtColor(image_gray, image, cv::COLOR_GRAY2BGR);

        for (int i = 0; i < (int)detections.size(); i++)
        {
            int color = 0; // everything ok (0 = green)
            if (!detections[i].good)
                color = 3; // yellow
            if (m_marker_db.m_marker.find(detections[i].id) == m_marker_db.m_marker.end())
            {
                color = 1; // not in db
            }
            else
            {
                if (detections[i].outlier)
                    color = 2; // outlier (red)
            }
            detections[i].draw(image, color, m_showResiduals);
        }
    }

    if (m_showUndist)
    {
        cv::Mat temp = image.clone();
        cv::undistort(temp, image, m_cameraMatrix, m_distCoeffs);
    }

    cv::imshow(WINDOWNAME, image); // OpenCV call
    return image;
}

/** Post processing: load images specified in config image_list attribute and
 * process them */
void CamTagNavApp::loadImages()
{
    cv::Mat image;
    int key;
    int idx = 0;

    /* iterate over all specified image file paths: */
    for (auto it = m_imgNames.begin(); it != m_imgNames.end(); it++)
    {
        image = cv::imread(*it);
        if (image.rows < 1 || image.cols < 1)
        {
            std::cout << "Could not load image " << *it << std::endl;
            continue;
        }
        idx++;

        /* image = */ processImage(image);
        // char output[128];
        // sprintf(output, "output_%05i.jpg", idx);
        // cv::imwrite(output, image);

        key = cv::waitKey(33);
        // while ((key = cv::waitKey(50)) == -1) {}
    }
}

// Video or image processing?
bool CamTagNavApp::isVideo()
{
    return m_imgNames.empty();
}

void CamTagNavApp::emitSolution(Eigen::Vector3d pos, Eigen::Quaterniond q)
{
    char send_buffer[1024];
    m_udpMessageCounter++;
    double timestamp_sec = 0.0; // FIXME implement this

    snprintf(send_buffer, sizeof(send_buffer),
            "%.3f, %u, %.4f, %.4f, %.4f, "
            "%.5f, %.5f, %.5f, %.5f",
            timestamp_sec,
            m_udpMessageCounter,
            pos(0), pos(1), pos(2),
            q.w(), q.x(), q.y(), q.z());

    m_udpsender.Send((unsigned char*)send_buffer, strlen(send_buffer));
}

