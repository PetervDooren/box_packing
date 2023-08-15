#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

cv::Scalar getRainbowColor(float value)
{
    float hue = (1 - value) * 240;  // Map value to hue range (blue to red)
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::cvtColor(hsv, hsv, cv::COLOR_HSV2BGR);
    return cv::Scalar(hsv.at<cv::Vec3b>(0, 0));
}

int main()
{
    // Create a RealSense pipeline and configure it to stream depth frames
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH);
    pipe.start(config);

    while (cv::waitKey(1) != 'q')
    {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // Get the width and height of the depth frame
        int width = depth_frame.get_width();
        int height = depth_frame.get_height();

        // Create a colorized depth image
        cv::Mat depth_image(cv::Size(width, height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat colorized_depth(cv::Size(width, height), CV_8UC3);

        // Find the minimum and maximum depth values
        float min_depth = std::numeric_limits<float>::max();
        float max_depth = std::numeric_limits<float>::min();
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float depth_value = depth_image.at<uint16_t>(y, x) / 1000.0f;
                min_depth = std::min(min_depth, depth_value);
                max_depth = std::max(max_depth, depth_value);
            }
        }

        // Apply rainbow coloring based on normalized depth values
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float depth_value = depth_image.at<uint16_t>(y, x) / 1000.0f;
                float normalized_depth = (depth_value - min_depth) / (max_depth - min_depth);
                cv::Scalar color = getRainbowColor(normalized_depth);
                colorized_depth.at<cv::Vec3b>(y, x) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }

        float dept_value_at = depth_image.at<uint16_t>(200, 200);

        // Display the colorized depth image
        cv::imshow("Colorized Depth Image", colorized_depth);
        // std::cout << dept_value_at<< std::endl;
    }

    // Stop the pipeline before exiting the program
    pipe.stop();

    return 0;
}
