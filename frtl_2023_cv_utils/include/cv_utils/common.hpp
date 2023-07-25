#ifndef COMMON_HPP
#define COMMON_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
/*
    TODO: remove dependency on highgui (may not work on raspiberry)
    Currently necessary for trackbar creation (write macro magic to
    only create trackbars when videoTest.hpp is included)
*/
#include <opencv2/highgui.hpp>
#include <vector>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#define TRACKBAR(member, windowName, processor, type, max) cv::createTrackbar(#member, windowName, NULL, max, \
    [](int val, void* instance) { \
            ((processor*)instance)->member = (type) val; \
    }, this);


//retirado de: https://json.nlohmann.me/features/arbitrary_types/
//(exemplo ns::person)
#define SAVE_TO_JSON(type) void type::save_to_json(std::string filename) { \
        std::ofstream out(filename); \
        nlohmann::json j = *(this); \
        out << std::setw(4) << j; \
    }

template<typename InputType, typename OutputType>
class BaseCVUtil
{
private:
    /* data */
public:
    virtual const OutputType apply(InputType input) = 0;
    virtual cv::Mat displayableResult(const cv::Mat rgbImage, OutputType processedData) = 0;
    virtual void createTrackbars() = 0;
    
    //definição precisa estar aqui - sim, eu testei colocar em um cpp, colocar à parte embaixo da função
    void displayProcessing(std::string inputFile="")  {
        cv::VideoCapture capture;
        if (!inputFile.empty())
            capture = cv::VideoCapture(inputFile);
        else
            capture = cv::VideoCapture(0);

        double fps = (double) capture.get(cv::VideoCaptureProperties::CAP_PROP_FPS);
        
        cv::namedWindow("raw video");
        cv::namedWindow("processed video");
        cv::Mat frame;
        bool paused = false;

        std::chrono::milliseconds last = 
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::system_clock::now()
                        .time_since_epoch());
        
        //relevante pro salvamento de parâmetros (input 's')
        int paramFileCounter = 0;

        while (1) {
        
            auto temp = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    std::chrono::system_clock::now()
                        .time_since_epoch());
        
            if (((temp - last).count() > 1e3/fps && !paused) || frame.empty()) {
                last = temp;
                capture >> frame;
            }
        
            if (!frame.empty()) {
                cv::Mat resized; 
                cv::resize(frame, resized, cv::Size(), 0.5, 0.5);
                cv::imshow("raw video", resized);
                cv::imshow("processed video", displayableResult(resized, apply(resized)));
            }
            else
                break;
        
            int key = cv::waitKey(1);
            if (key == 'p' || key == 'P')
                paused = !paused;
            if (key == 'n' || key == 'N')
                capture >> frame;
            if (key == 's' || key == 'S') {
                std::string filename = "tuned_parameters";
                filename.append(std::to_string(paramFileCounter++)).append(".json");
                save_to_json(filename);
            }
            if (key == 'q' || key == 'Q')
                break;
        }
        
        cv::destroyWindow("processed video");
        cv::destroyWindow("raw video");
        cv::waitKey(1);
    }
    
    void saveProcessing(std::string outputFile, int Nframes, std::string inputFile="")  {
        cv::VideoCapture capture;
        if (!inputFile.empty())
            capture = cv::VideoCapture(inputFile);
        else{
            capture = cv::VideoCapture(0, cv::CAP_V4L2);
            capture.set(cv::CAP_PROP_FPS, 30);
        }

        int height = capture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT);
        int width  = capture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH);
        cv::VideoWriter writer(outputFile,
                    cv::VideoWriter::fourcc('M','J','P','G'),
                    30, {width, height});
        if (!writer.isOpened())
        {
            std::cout << "não abriu" << std::endl;
        }
        int frameCount = 0;
        cv::Mat frame;
        while (1) {
            capture >> frame;
            std::cout << !frame.empty() << " " << frameCount << std::endl;
            if (!frame.empty() && (frameCount++) <= Nframes) {
                writer << displayableResult(frame, apply(frame));
            }
            else
                break;
        }
        
    }

    /*
        Brief de implementação: usar SAVE_TO_JSON definido acima para gerar a implementação
        correta automaticamente. Para usar estas funções o macro NLOHMANN_DEFINE_TYPE_INTRUSIVE
        deve estar definido para o tipo concreto (https://github.com/nlohmann/json)
    */
    virtual void save_to_json(std::string filename) = 0;
};

/*
    Brief de chamada: T x = load_from_json<T>(filename);
*/
template<typename T>
T load_from_json(std::string filename) {
    std::ifstream in(filename);
    nlohmann::json j;
    in >> j;
    T ret;
    j.get_to<T>(ret);
    return ret;
}

typedef std::vector<cv::Point2i> Contour;
typedef std::vector<Contour> ContourList;

#endif