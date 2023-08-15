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

        std::cout << "loop" << std::endl;
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Get the depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        int width = depth_frame.get_width();
        int height = depth_frame.get_height();

        std::cout <<"depth: "<< width << " & " << height<< std::endl;

        // Convert the color frame to an OpenCV matrix
        cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
     

    // Display the image
    // cv::imshow("Blue Pixel", image);
    // cv::waitKey(0);
        // Convert the depth frame to an OpenCV matrix
        cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(width, height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat colorized_depth;

    // Convert the depth image to an 8-bit format for color mapping
        depth_image.convertTo(colorized_depth, CV_8UC1, 255.0 / 1000.0);

    // Apply the color map
        cv::applyColorMap(colorized_depth, colorized_depth, cv::COLORMAP_JET);

        
       

        // std::cout<<depth_image.size()<<std::endl;
        // std::cout<<color_image.size()<<std::endl;


        // Detect ArUco markers in the color image
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(color_image, dictionary, marker_corners, marker_ids);
        
        // Draw the detected markers on the color image
        if (marker_ids.size() > 0)
            cv::aruco::drawDetectedMarkers(color_image, marker_corners, marker_ids);

    //   for (int i=-50; i<=50; i++){
    //         depth_image.at<cv::Vec3b>(100+i, 100+i) = cv::Vec3b(255, 0, 0);
    //     }

        // Display the color image
        cv::imshow("Color Image", color_image);

        // Display the depth image (optional)
        cv::imshow("Depth Image", depth_image);

        cv::imshow("Colorized Depth Image", colorized_depth);

        float depth_value[width][height];

        for (int i = 0; i<width; i++){
            for (int ii = 0; i<height; i++){
                depth_value[i][ii] = depth_frame.get_distance(i,ii);
                std::cout << depth_value[i][ii] << " " ; 

            }
            std::cout<< std::endl;
        }

        // Calculate and display the distance to each detected marker
        // for (size_t i = 0; i < marker_ids.size(); ++i) {


        //     int marker_id = marker_ids[i];
        //     std::vector<cv::Point2f> marker_corner = marker_corners[i];

        //     // Calculate the center of the marker
        //     cv::Point2f marker_center(0, 0);
        //     for (const cv::Point2f& corner : marker_corner) {
        //         marker_center += corner;
        //     }
        //     marker_center /= 4.0;

        //     cv::Point marker_center_int(static_cast<int>(marker_center.x), static_cast<int>(marker_center.y));

        //     std::cout << i << "=" << marker_center.x << " " << marker_center.y << std::endl;

        //     // Get the depth value at the center of the marker


        //     // // Convert the depth value to meters using the depth scale of the camera
        //     // float depth_scale = selected_device.first<rs2::depth_sensor>().get_depth_scale();
        //     // float distance = depth_value * depth_scale;

        //     // // Display the distance in the terminal
        //     // std::cout << "Marker ID: " << marker_id << ", Distance: " << distance << " meters" << std::endl;
        // }

        // Exit the loop if the 'Esc' key is pressed
        if (cv::waitKey(1) == 27)
            break;
    }

    // Stop the pipeline
    pipe.stop();

    return 0;
}
