#include "cv_utils/qrcode_reader.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

QRCodeReader::QRCodeReader() {
    // Constructor code, if needed.
}

QRCodeReader::~QRCodeReader() {
    // Destructor code, if needed.
}

void QRCodeReader::detectQRCode(const cv::Mat& image, bool& got_qrcode) {
    cv::QRCodeDetector qrCodeDetector;

    std::vector<cv::Point> qrCodeCorners;
    std::string decodedInfo;

    got_qrcode = false; // Initialize to false.

    // Detect QR code in the image
    std::vector<cv::Point> points;
    qrCodeDetector.detect(image, points);

    // Check if a QR code is found
    if (points.size() > 0) {
        qrCodeCorners = points;

        // Extract and decode the QR code
        decodedInfo = qrCodeDetector.decode(image, points);

        // If decoding is successful, set got_qrcode to true
        if (!decodedInfo.empty()) {
            got_qrcode = true;

            // Display the decoded information
            std::cout << "QR Code Content: " << decodedInfo << std::endl;
        }
    }
}
