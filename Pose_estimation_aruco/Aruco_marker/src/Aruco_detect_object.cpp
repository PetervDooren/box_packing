#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    // Create a RealSense pipeline and configure it to stream color and depth frames
    rs2::pipeline pipe;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH);
    pipe.start(config);

    // Create a cascade classifier for object detection (change the path to the XML file accordingly)
    cv::CascadeClassifier cascade;
    cascade.load("cascade_xml_file.xml");

    while (cv::waitKey(1) != 'q')
    {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Convert the color frame to an OpenCV image
        cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the color image to grayscale for object detection
        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        // Equalize the histogram of the grayscale image to improve object detection
        cv::equalizeHist(gray, gray);

        // Detect objects in the grayscale image using the cascade classifier
        std::vector<cv::Rect> objects;
        cascade.detectMultiScale(gray, objects, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

        // Draw a green bounding box around each detected object
        for (const auto& rect : objects)
        {
            cv::rectangle(color, rect, cv::Scalar(0, 255, 0), 2);
        }

        // Display the color image with bounding boxes
        cv::imshow("Object Detection", color);
    }

    // Stop the pipeline before exiting the program
    pipe.stop();

    return 0;
}
