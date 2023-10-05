#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <cmath>

#include <iostream>
#include <string>
#include "cv_utils/videoTest.hpp"
#include "cv_utils/ColorBoxFilter.hpp"
#include "cv_utils/ContourExtractor.hpp"
// remover elipses concentricas?
#include "cv_utils/EllipseFitter.hpp"
#include "cv_utils/PolygonIdentifier.hpp"
#include "cv_utils/PointCloudTracker.hpp"
#include "cv_utils/ImageToWorldConverter.hpp"
#include <opencv2/core.hpp>