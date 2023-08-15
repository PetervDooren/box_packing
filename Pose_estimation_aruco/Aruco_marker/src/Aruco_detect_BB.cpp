#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>       //OpenCV version: 4.8.0-dev

int main() {

  // Create a Pipeline - this serves as a top-level API for streaming and
  // processing frames
  rs2::pipeline p;

  // Configure and start the pipeline
  p.start();

  while (true) {

    // Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame();
    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    int width = color_frame.get_width();
    int height = color_frame.get_height();

    cv::Mat inputImage(cv::Size(width, height), CV_8UC3,
                       (void *)color_frame.get_data());

    // Detect markers

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams =
        cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(inputImage, markerCorners, markerIds,
                           rejectedCandidates);

    // Draw markers

    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    // Cameramatrix & distortion matrix: verkegen door checkerboard calibratie
    // te doen, zie calibratie mapje
    cv::Mat cameraMatrix =
        (cv::Mat_<float>(3, 3) << 991.6899044146527, 0, 749.3129464107503, 0,
         970.1750217910793, 482.609418075871, 0.0, 0.0, 1.0);
    cv::Mat distCoeffs =
        (cv::Mat_<float>(5, 1) << 0.1520171287384349, -0.1450585941447574,
         0.04365948290341132, 0.04198055042210847, 0.05629158652386724);

    float markerSize = 0.058; // Afmeting van Marker in m

    if (!markerCorners.empty()) {
      std::vector<cv::Vec3d> tvec_sum;
      std::vector<cv::Vec3d> rvec_sum;

      for (size_t i = 0; i < markerCorners.size();
           ++i) { // markerCorners.size() is gelijk aan de hoeveelheid markers

        std::cout << "Marker corners: " << markerCorners.size() << std::endl;

        // Define object points for the marker's corners
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(0, 0, 0));
        objectPoints.push_back(cv::Point3f(markerSize, 0, 0));
        objectPoints.push_back(cv::Point3f(markerSize, markerSize, 0));
        objectPoints.push_back(cv::Point3f(0, markerSize, 0));

        // Convert 2D image points to the correct format
        std::vector<cv::Point2f> imagePoints;
        std::vector<cv::Scalar> cornerColors = {
            cv::Scalar(255, 0, 0),  // Blue
            cv::Scalar(0, 255, 0),  // Green
            cv::Scalar(0, 0, 255),  // Red
            cv::Scalar(0, 255, 255) // Yellow
        };

        for (const auto &corner : markerCorners[i]) {
          imagePoints.push_back(corner);
        }

        std::cout << "Objectpoints: " << objectPoints << std::endl;
        std::cout << "imagePoints: " << imagePoints << std::endl;
        // check de GESCHREVEN documentatie en niet de uitgebeelde documentatie
        // Estimate pose using solvePnP

        cv::Vec3d rvec, tvec; // std::vector<cv::Vec3d> rvec; //Dit was een
                              // error veroorzaker......
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,
                     tvec); // P_camera = RCA(rvec) * P_marker + t(tvec); RCA
                            // equals rotation matrix from aruco to camera

        std::cout << "Rvec: " << rvec << std::endl;
        std::cout << "Tvec: " << tvec << std::endl;

        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec,
                          markerSize * 0.5);

        // Tekenen van markerpunten en assen op de uitvoerafbeelding
        // Oorspronkelijke hoekpunten van de marker
        cv::Point2f p0 = markerCorners[i][0];
        cv::Point2f p1 = markerCorners[i][1];
        cv::Point2f p2 = markerCorners[i][2];
        cv::Point2f p3 = markerCorners[i][3];

        tvec_sum.push_back(tvec);
        rvec_sum.push_back(rvec);

        for (const auto &tvec_show : tvec_sum) {
          std::cout << tvec_show << " ";
        }
        std::cout << std::endl;
      }

      std::vector<int> marker_idx;
      int marker1 = 42;
      int marker2 = 5;

      std::cout << "Marker Id's: " << std::endl;
      for (int i = 0; i < markerIds.size(); ++i) {
        std::cout << markerIds[i] << " ";
      }
      std::cout << std::endl;

      for (int i = 0; i < markerCorners.size(); ++i) {
        if (markerIds[i] == marker1) {
          marker_idx.push_back(i);
        }
      }

      for (int i = 0; i < markerCorners.size(); ++i) {
        if (markerIds[i] == marker2) {
          marker_idx.push_back(i);
        }
      }

      std::cout << "Marker index:"
                << " ";

      for (int i = 0; i < marker_idx.size(); ++i) {
        std::cout << marker_idx[i] << " ";
      }

      std::cout << std::endl;

      std::cout << "Marker index: " << marker_idx.size() << std::endl;

      if (markerCorners.size() > 1) {

        if (marker_idx.size() > 1) {

          for (int i = 0; i < (marker_idx.size() - 1); i++) {

            int v = marker_idx[i];
            int v_pl_1 = marker_idx[i + 1];

            // Rotation of aruco 2 to aruco 1: P_aruco1 = R_A1C^T*R_CA2*P_aruco2
            // where Ai and C equal aruco i and camera.
            cv::Mat R_A1A2;
            cv::Mat R_CA1;
            cv::Mat R_CA2;
            cv::Rodrigues(rvec_sum[v], R_CA1);
            cv::Rodrigues(rvec_sum[v_pl_1], R_CA2);

            std::cout << "Rvec_sum: "
                      << " ";
            for (int i = 0; i < rvec_sum.size(); ++i) {
              std::cout << rvec_sum[i] << "";
            }
            std::cout << std::endl;

            R_A1A2 = R_CA1.inv() *
                     R_CA2; // Inverse of R_CA1, since the pose of the second
                            // marker in the first marker's frame is desired.

            std::cout << "Rotation C A1: " << R_CA1 << std::endl;
            std::cout << "Rotation C A2: " << R_CA2 << std::endl;

            cv::Vec3d rvec_euler;
            cv::Rodrigues(R_A1A2, rvec_euler);
            double roll = rvec_euler[0];
            double pitch = rvec_euler[1];
            double yaw = rvec_euler[2];

            double angle_x = roll * 180.0 / CV_PI;
            double angle_y = pitch * 180.0 / CV_PI;
            double angle_z = yaw * 180.0 / CV_PI;

            std::cout << "Rotation difference between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " around X-axis: " << angle_x << " degrees"
                      << std::endl;
            std::cout << "Rotation difference between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " around Y-axis: " << angle_y << " degrees"
                      << std::endl;
            std::cout << "Rotation difference between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " around Z-axis: " << angle_z << " degrees"
                      << std::endl;

            cv::Mat tvec_diff = R_CA1.inv() * (cv::Mat(tvec_sum[v_pl_1]) -
                                               cv::Mat(tvec_sum[v]));

            double distance_x_in_marker_frame = tvec_diff.at<double>(
                0); // distance in x is 0, distance in y is 1, distance z is 2
            std::cout << "Distance in x between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " in marker frame: " << distance_x_in_marker_frame
                      << std::endl;

            double distance_y_in_marker_frame = tvec_diff.at<double>(
                1); // distance in x is 0, distance in y is 1, distance z is 2
            std::cout << "Distance in y between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " in marker frame: " << distance_y_in_marker_frame
                      << std::endl;

            double distance_z_in_marker_frame = tvec_diff.at<double>(
                2); // distance in x is 0, distance in y is 1, distance z is 2
            std::cout << "Distance in z between marker " << markerIds[v]
                      << " and marker " << markerIds[v_pl_1]
                      << " in marker frame: " << distance_z_in_marker_frame
                      << std::endl;
          }

        } else {
          std::cout << "Only 1 marker for pose estimation is detected"
                    << std::endl;
        }

        for (int i = 1; i < (markerCorners.size()); i++) {
          double distance_x = tvec_sum[i][0] - tvec_sum[i - 1][0];
          std::cout << distance_x << std::endl;
        }

      } else {
        std::cout << "Only 1 marker is detected" << std::endl;
      }

      // }

    } else {
      std::cout << "No marker detected" << std::endl;
    }

    // Toon de uitvoerafbeelding met gedetecteerde markers en assen
    cv::imshow("out", outputImage);

    // Wacht op gebruikerstoets om het venster te sluiten
    if (cv::waitKey(1) == 'q')
      break;
  }
}
