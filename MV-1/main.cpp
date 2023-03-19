#include <opencv2/opencv.hpp>
#include "HomogeneousBgDetector.h"
#include <cmath>
#include <math.h> 
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

cv::Point2f point1(-1, -1);
cv::Point2f point2(-1, -1);
bool clicked = false;

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (!clicked) {
            point1 = cv::Point2f(x, y);
            clicked = true;
        }
        else {
            point2 = cv::Point2f(x, y);
            clicked = false;
        }
    }
}


// Returns the Euclidean distance between two points.
float distance(cv::Point2f pt1, cv::Point2f pt2) {
    float dx = pt1.x - pt2.x;
    float dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// Orders a set of 4 points in a consistent order such that the top-left point is
// the first entry in the list, followed by top-right, bottom-right, and bottom-left.
void   order_points(cv::Point2f points[4], cv::Point2f points_out[4]) {
    cv::Point2f ordered_points[4];

    // Compute the sum and difference of the (x, y) coordinates for each point.
    std::vector<float> sums, diffs;
    for (int i = 0; i < 4; i++) {
        sums.push_back(points[i].x + points[i].y);
        diffs.push_back(points[i].x - points[i].y);
    }

    // The top-left point will have the smallest sum, whereas the bottom-right point
    // will have the largest sum.
    int tl_idx = 0, br_idx = 0;
    for (int i = 0; i < 4; i++) {
        if (sums[i] < sums[tl_idx]) {
            tl_idx = i;
        }
        if (sums[i] > sums[br_idx]) {
            br_idx = i;
        }
    }

    // The top-right point will have the smallest difference, whereas the bottom-left
    // point will have the largest difference.
    int tr_idx = 0, bl_idx = 0;
    for (int i = 0; i < 4; i++) {
        if (diffs[i] < diffs[tr_idx]) {
            tr_idx = i;
        }
        if (diffs[i] > diffs[bl_idx]) {
            bl_idx = i;
        }
    }

    // Reconstruct the ordered points list.
    points_out[0] = points[tl_idx];
    points_out[1] = points[tr_idx];
    points_out[2] = points[br_idx];
    points_out[3] = points[bl_idx];

}

double calculateDistance(int x1, int y1, int x2, int y2) {
    double dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    return dist;
}

cv::Point2d midpoint(cv::Point2d ptA, cv::Point2d ptB) {
    return cv::Point2d((ptA.x + ptB.x) * 0.5, (ptA.y + ptB.y) * 0.5);
}

double calculateDistanceFromPoint2d(cv::Point2d ptA, cv::Point2d ptB) {
    double dist = sqrt(pow(ptA.x - ptB.x, 2) + pow(ptA.y - ptB.y, 2));
    return dist;
}

int main() {

    double width = 6, height = 3;


    try
    {
        string input;
        cout << "Enter width & height of reference object in cm (width,height) eg:- 4,5 : ";
        getline(cin, input);

        stringstream ss(input);

        // Use getline to split the string by the comma delimiter
        string token;
        getline(ss, token, ',');
        width = stod(token);
        getline(ss, token, ',');
        height = stod(token);
    }
    catch (const std::exception&)
    {
        throw ("invalid input");
    }



    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Unable to open camera\n";
        return -1;
    }

    HomogeneousBgDetector detector;

    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera", 640, 480);
    cv::setMouseCallback("Camera", onMouse, 0);

    cv::Mat frame;
    while (true) {
        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "Unable to capture frame\n";
            break;
        }

        std::vector<std::vector<cv::Point>> contours = detector.detect_objects(frame);

        double ref_object_px_per_cm = 0;

        for (int i = 0; i < contours.size(); i++) {
            if (ref_object_px_per_cm != 0)
                continue;

            cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            cv::Point2f box[4];
            rect.points(box);

            cv::Point2f box_ordered[4];
            order_points(box, box_ordered);

            for (int j = 0; j < 4; j++) {
                cv::circle(frame, box_ordered[j], 5, cv::Scalar(0, 0, 255), -1);
                cv::line(frame, box_ordered[j], box_ordered[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
            }

            double w = rect.size.width;
            double h = rect.size.height;

            cv::putText(frame, "Width " + std::to_string(round(w)), cv::Point(rect.center.x + 15, rect.center.y + 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(100, 200, 0), 2);
            cv::putText(frame, "Height " + std::to_string(round(h)), cv::Point(rect.center.x - 15, rect.center.y - 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(100, 200, 0), 2);

            cv::Point2f tl = box_ordered[0];
            cv::Point2f bl = box_ordered[3];
            cv::Point2f tr = box_ordered[1];
            cv::Point2f br = box_ordered[2];

            cv::Point2f tlbl = midpoint(tl, bl);
            cv::Point2f trbr = midpoint(tr, br);

            double ref_object_px = cv::norm(tlbl - trbr);

            if (ref_object_px != 0 && width != 0)
                ref_object_px_per_cm = ref_object_px / width;

            cv::putText(frame, "Reference Object pixels/cm length " + std::to_string(ref_object_px_per_cm), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0), 2);
        }

        if (clicked) {
            cv::circle(frame, point1, 3, cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, "Point 1", point1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        if (point1 != cv::Point2f(-1, -1) && point2 != cv::Point2f(-1, -1)) {
            cv::line(frame, point1, point2, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, "Point 2", point2, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

            if (ref_object_px_per_cm != 0) {
                float dist = calculateDistanceFromPoint2d(point1, point2) / ref_object_px_per_cm;
                std::ostringstream dist_str;
                dist_str << "Distance (cm): " << dist;
                cv::putText(frame, dist_str.str(), cv::Point(450, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            }

        }

        cv::imshow("Camera", frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}