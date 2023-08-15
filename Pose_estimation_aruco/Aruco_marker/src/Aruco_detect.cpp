#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main() {
    rs2::context ctx;
    
    rs2::device_list devices = ctx.query_devices();

    if (devices.size() == 0) {
        std::cout << "No RealSense devices detected" << std::endl;
        return 0;
    }

    std::cout << "test" << std::endl;

    rs2::device selected_device;
    for (rs2::device device : devices) {
        std::string device_name = device.get_info(RS2_CAMERA_INFO_NAME);
        if (device_name.find("L515") != std::string::npos) {
            selected_device = device;
            break;
        }
    }

    if (!selected_device) {
        std::cout << "No RealSense L515 device found" << std::endl;
        return 0;
    }

    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 0);
    config.enable_stream(RS2_STREAM_DEPTH, 0);

    // Start the pipeline with the configuration
    rs2::pipeline pipe;
    pipe.start(config);


    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

    while (true) {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Get the depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // Convert the color frame to an OpenCV matrix
        cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the depth frame to an OpenCV matrix
        cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Detect ArUco markers in the color image
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(color_image, dictionary, marker_corners, marker_ids);
        
        // Draw the detected markers on the color image
        if (marker_ids.size() > 0)
            cv::aruco::drawDetectedMarkers(color_image, marker_corners, marker_ids);

        // Display the color image
        cv::imshow("Color Image", color_image);

        for (auto id : marker_ids) {
            std::cout << "Marker ID: " << id << std::endl;
        }

        for (auto corner : marker_corners) {
            std::cout << "Marker corners: " << corner << std::endl;
        }

        
        // Exit the loop if the 'Esc' key is pressed
        if (cv::waitKey(1) == 27)
            break;
    }



    // Stop the pipeline
    pipe.stop();

    return 0;
}
