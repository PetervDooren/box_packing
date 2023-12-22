#include "aruco_detector.h"

#include <opencv2/opencv.hpp> //OpenCV version: 4.8.0-dev
#include <opencv2/aruco.hpp>
#include <Eigen/Geometry>

ArucoDetector::ArucoDetector()
{
    // Create a Pipeline - this serves as a top-level API for streaming and
    // processing frames
    rs2::pipeline p;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8,30); //IMPORTANT to calibrate the camera this way.......
    // Configure and start the pipeline
    p.start(config);
}

bool ArucoDetector::getPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    std::vector<float> poseVec;

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
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // Draw markers
    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    cv::Mat cameraMatrix =
        (cv::Mat_<float>(3, 3) << 606.7253166158411, 0, 328.5626555595404, 0,
         610.7115630742445, 256.9858829373862, 0.0, 0.0, 1.0);

    cv::Mat distCoeffs =
        (cv::Mat_<float>(5, 1) << 0.09768923357470966, 0.06632156915029483,
         -0.0009197939963682129, 0.003442722249108425, -0.938625280999511);
    // float markerSize = 0.16; // Afmeting van Marker in m

    if (!markerCorners.empty()) {
      std::vector<cv::Vec3d> tvec_sum;
      std::vector<cv::Vec3d> rvec_sum;

      for (size_t i = 0; i < markerCorners.size();
           ++i) { // markerCorners.size() is gelijk aan de hoeveelheid markers

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


        // check de GESCHREVEN documentatie en niet de uitgebeelde documentatie
        // Estimate pose using solvePnP

        cv::Vec3d rvec, tvec; // std::vector<cv::Vec3d> rvec; //Dit was een
                              // error veroorzaker......
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,
                     tvec); // P_camera = RCA(rvec) * P_marker + t(tvec); RCA
                            // equals rotation matrix from aruco to camera

        cv::Mat rotationMatrix;
        cv::Rodrigues(rvec, rotationMatrix);

        cv::Mat R_CA; //rotation matrix from aruco to camera: Frame_cam = R_CA * Frame_aruco
        cv::Rodrigues(rvec, R_CA);
        cv::Mat rvec_desired = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0); // Desired rotation in ArUco frame
        cv::Mat rvec_des_cam_frame;
        cv::Mat tvec_desired = (cv::Mat_<double>(3, 1) << -markerSize/2, -markerSize/2, 0.30); // Desired rotation in ArUco frame
        cv::Mat tvec_des_cam_frame;

        rvec_des_cam_frame = R_CA*rvec_desired; // rotation of desired pose in camera frame
        tvec_des_cam_frame = R_CA*tvec_desired; // translation of aruco-desired_pose in camera_frame

        cv::Rodrigues(rvec - rvec_desired, rotationMatrix);
        double x_roll = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
        double y_pitch = atan2(-rotationMatrix.at<double>(2, 0), sqrt(rotationMatrix.at<double>(2, 1) * rotationMatrix.at<double>(2, 1) + rotationMatrix.at<double>(2, 2) * rotationMatrix.at<double>(2, 2)));
        double z_yaw = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));

        double degrees_per_radian = 180.0 / M_PI; // Conversion factor
        double x_roll_deg = x_roll * degrees_per_radian;
        double y_pitch_deg = y_pitch * degrees_per_radian;
        double z_yaw_deg = z_yaw * degrees_per_radian;

        std::cout<<"X deg: " << x_roll_deg << std::endl;
        std::cout<<"Y deg: " << y_pitch_deg << std::endl;
        std::cout<<"Z deg: " << z_yaw_deg << std::endl;

        cv::Mat tvec_diff = tvec - tvec_desired;
        std::cout << "Tvec_ diff: " << tvec_diff << std::endl;

        poseVec.push_back(x_roll);
        poseVec.push_back(y_pitch);
        poseVec.push_back(z_yaw);
        poseVec.push_back(tvec_diff.at<double>(0));
        poseVec.push_back(tvec_diff.at<double>(1));
        poseVec.push_back(tvec_diff.at<double>(2));

        // Calculate the marker's position in the image center
        cv::Point2f imageCenter(width / 2, height / 2);
        cv::Point2f markerCenter = (markerCorners[i][0] + markerCorners[i][2]) * 0.5;

        // Calculate the displacement of the marker from the image center
        float displacementX = markerCenter.x - imageCenter.x;
        float displacementY = markerCenter.y - imageCenter.y;
      
        cv::circle(outputImage, imageCenter, 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(outputImage, markerCenter, 5, cv::Scalar(0, 0, 255), -1);

        //Determine if the robot should move closer or further away from aruco --> aruco should 'perfectly' fit in the 
        int min_y_pixel = 480;
        int max_y_pixel = 0;
        cv::Point2f pl1;
        cv::Point2f pl2;

        for (int i = 0; i<imagePoints.size(); i++){

            if(imagePoints[i].y > max_y_pixel){
              max_y_pixel = imagePoints[i].y;
              pl1.x=imagePoints[i].x;
              pl1.y=imagePoints[i].y;
            }
            if(imagePoints[i].y < min_y_pixel){
              min_y_pixel = imagePoints[i].y;
              pl2.x = imagePoints[i].x;
              pl2.y = imagePoints[i].y;
            }             
        }

        cv::Point2f ptest(640-50,200);
        cv::circle(outputImage, ptest, 5, cv::Scalar(255, 0, 0), -1);

        int thickness = 2; 
        
          // Line drawn using 8 connected 
          // Bresenham algorithm 
        cv::line(outputImage, pl1, pl2, cv::Scalar(255, 0, 0), thickness, cv::LINE_8); 

        cv::Point2f psq1(50,50);
        cv::Point2f psq2(50,430);
        cv::Point2f psq3(590,430);
        cv::Point2f psq4(590,50);
        
        cv::line(outputImage, psq1, psq2, cv::Scalar(0, 255, 0), thickness, cv::LINE_8);
        cv::line(outputImage, psq2, psq3, cv::Scalar(0, 255, 0), thickness, cv::LINE_8);
        cv::line(outputImage, psq3, psq4, cv::Scalar(0, 255, 0), thickness, cv::LINE_8);
        cv::line(outputImage, psq4, psq1, cv::Scalar(0, 255, 0), thickness, cv::LINE_8); 

        // Tekenen van markerpunten en assen op de uitvoerafbeelding
        // Oorspronkelijke hoekpunten van de marker
        cv::Point2f p0 = markerCorners[i][0];
        cv::Point2f p1 = markerCorners[i][1];
        cv::Point2f p2 = markerCorners[i][2];
        cv::Point2f p3 = markerCorners[i][3];

        cv::circle(outputImage, p0, 5, cv::Scalar(255, 0, 0), -1);



        tvec_sum.push_back(tvec);
        rvec_sum.push_back(rvec);
      }

      std::vector<int> marker_idx;
      //   int marker1 = 42;
      //   int marker2 = 5;

      for (int i = 0; i < markerCorners.size(); ++i) {
        if (markerIds[i] == markerId) {
          marker_idx.push_back(i);
        }
      }

    } else {
      std::cout << "No marker detected" << std::endl;
    }

    // Toon de uitvoerafbeelding met gedetecteerde markers en assen
    cv::imshow("out", outputImage);

    // Wacht op gebruikerstoets om het venster te sluiten
    //if (cv::waitKey(1) == 'q')

    //return poseVec;
    return true;
}

/*
int main() {

  // Create a Pipeline - this serves as a top-level API for streaming and
  // processing frames
  rs2::pipeline p;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8,30); //IMPORTANT to calibrate the camera this way.......
  // Configure and start the pipeline
  p.start(config);

  while (true) {
    int markerId1 = 42; // Vervang door je marker-ID's
    float markerSize = 0.10;

    std::vector<float> Posevec = calculatePose(p, markerId1, markerSize);

    std::cout << "Pose: " << std::endl;

    std::ofstream outputFile("data.txt");
    if (outputFile.is_open()) {
        for (int i = 0; i < Posevec.size(); ++i) {
            outputFile << Posevec[i] << std::endl;
        }
        outputFile.close();
    }

    for (int i = 0; i < Posevec.size(); ++i) {
      // if (i == 0) {
      //   std::cout << "Rotation X: " << Posevec[i] << std::endl;
      // }
      // if (i == 1) {
      //   std::cout << "Rotation Y: " << Posevec[i] << std::endl;
      // }

      // if (i == 2) {
      //   std::cout << "Rotation Z: " << Posevec[i] << std::endl;
      // }

      if (i == 0) {
        std::cout << "Rot X: " << Posevec[i]*180.0 / M_PI << std::endl;
      }

      if (i == 1) {
        std::cout << "Rot Y: " << Posevec[i]*180.0 / M_PI << std::endl;
      }

      if (i == 2) {
        std::cout << "Rot Z: " << Posevec[i]*180.0 / M_PI << std::endl;
      }
      
      if (i == 3) {
        std::cout << "Trans X: " << Posevec[i] << std::endl;
      }

      if (i == 4) {
        std::cout << "Trans Y: " << Posevec[i] << std::endl;
      }

      if (i == 5) {
        std::cout << "Trans Z: " << Posevec[i] << std::endl;
      }

      // if (i == 5) {
      //   std::cout << "Distance Z: " << Posevec[i] << std::endl;
      // }

      // std::cout << Posevec[i] << " ";
    }
 
    std::cout << std::endl;

    // Afmeting rijstpak. Breedte: 13.5 cm & Hoogte: 19.5 cm
    // Plaatsing aruco op rijstpak: 6cm van bovenkant & 4cm van zijkant
    // Plaatsing vanuit midden: zijkant: 13.5/2-4 = 2.75 cm bovenkant:19.5/2-6
    // = 3.75 cm
  }

  return 0;
}
*/