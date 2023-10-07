#ifndef QRCODE_READER_HPP
#define QRCODE_READER_HPP

#include <opencv2/opencv.hpp>
#include <string>

class QRCodeReader {
public:
    QRCodeReader();
    ~QRCodeReader();

    void detectQRCode(const cv::Mat& image, bool& got_qrcode);

private:
    // You can include any private members or helper functions here.
};

#endif // QRCODE_READER_HPP
