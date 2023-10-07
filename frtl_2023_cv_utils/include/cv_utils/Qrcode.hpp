#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

data qrCode(const cv::Mat &img) {

        cv::QRCodeDetector qrCodeDetector;
        std::string qrCodeData = qrCodeDetector.detectAndDecode(img);

        return qrCodeData;
}