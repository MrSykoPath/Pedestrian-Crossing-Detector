#include "Utilities.h"
#define ADAPTIVE_THRESH_KERNEL_SIZE 89
#define MIN_AREA 400
#define LOWER_RATIO 1.20
#define UPPER_RATIO 1.80
#define RHO_THRESHOLD 6
#define THETA_THRESHOLD 0.2
#define HOUGH_THRESHOLD 70
#define RECTANGULARITY_THRESHOLD 0.55
#define CIRCULARITY_THRESHOLD 0.9
#define NUM_IMAGES 20
#define CLIP_LIMIT 1.0
#define MEDIAN_FILTER_SIZE_1 5
#define MEDIAN_FILTER_SIZE_2 5
#define OPENING_SIZE 9.0
// Ground truth for pedestrian crossings. Each row contains
// 1. the image number (PC?.jpg)
// 2. the coordinates of the line at the top of the pedestrian crossing (left column, left row, right column, right row)
// 3. the coordinates of the line at the bottom of the pedestrian cross ng (left column, left row, right column, right row)
int pedestrian_crossing_ground_truth[][9] = {
    {10, 0, 132, 503, 113, 0, 177, 503, 148},
    {11, 0, 131, 503, 144, 0, 168, 503, 177},
    {12, 0, 154, 503, 164, 0, 206, 503, 213},
    {13, 0, 110, 503, 110, 0, 156, 503, 144},
    {14, 0, 95, 503, 104, 0, 124, 503, 128},
    {15, 0, 85, 503, 91, 0, 113, 503, 128},
    {16, 0, 65, 503, 173, 0, 79, 503, 215},
    {17, 0, 43, 503, 93, 0, 89, 503, 146},
    {18, 0, 122, 503, 117, 0, 169, 503, 176},
    {20, 0, 157, 503, 131, 0, 223, 503, 184},
    {21, 0, 140, 503, 136, 0, 190, 503, 183},
    {22, 0, 114, 503, 97, 0, 140, 503, 123},
    {23, 0, 133, 503, 122, 0, 198, 503, 186},
    {24, 0, 107, 503, 93, 0, 146, 503, 118},
    {25, 0, 58, 503, 164, 0, 71, 503, 204},
    {26, 0, 71, 503, 131, 0, 106, 503, 199},
    {27, 0, 138, 503, 151, 0, 179, 503, 193}};
void MyApplication()
{
    // Load the pedestrian crossing images
    Mat original_image[NUM_IMAGES];
    for (int image_index = 10; (image_index < NUM_IMAGES + 10); image_index++)
    {
        char filename[200];
        sprintf(filename, "Media/PC%d.jpg", image_index);
        original_image[image_index - 10] = imread(filename, -1);
        if (original_image[image_index - 10].empty())
        {
            cerr << "Error: Could not load image " << filename << endl;
        }
    }
    // Apply Median filter to remove noise
    Mat median_filtered_image1[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        medianBlur(original_image[image_index], median_filtered_image1[image_index], MEDIAN_FILTER_SIZE_1);
        // namedWindow("Original Image", WINDOW_AUTOSIZE);
        // imshow("Original Image", original_image[image_index]);
        // namedWindow("Median Filtered Image1", WINDOW_AUTOSIZE);
        // imshow("Median Filtered Image1", median_filtered_image1[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Apply Clahe to enhance the image
    vector<Mat> channels(3);
    Mat equalized_image[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        cvtColor(median_filtered_image1[image_index], equalized_image[image_index], COLOR_BGR2HLS);
        split(equalized_image[image_index], channels);
        Ptr<CLAHE> clahe = createCLAHE(CLIP_LIMIT, Size(8, 8));
        clahe->apply(channels[1], channels[1]);
        merge(channels, equalized_image[image_index]);
        cvtColor(equalized_image[image_index], equalized_image[image_index], COLOR_HLS2BGR);
        // namedWindow("Equalized Image", WINDOW_AUTOSIZE);
        // imshow("Equalized Image", equalized_image[image_index]);
        // namedWindow("Median Filtered Image1", WINDOW_AUTOSIZE);
        // imshow("Median Filtered Image1", median_filtered_image1[image_index]);
        // waitKey(0);
        // destroyAllWindows();
        channels.clear();
    }
    // Convert the images to grayscale
    Mat gray_image[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        cvtColor(equalized_image[image_index], gray_image[image_index], COLOR_BGR2GRAY);
        // namedWindow("Gray Image", WINDOW_AUTOSIZE);
        // imshow("Gray Image", gray_image[image_index]);
        // namedWindow("Equalized", WINDOW_AUTOSIZE);
        // imshow("Equalized", equalized_image[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Remove Noise using Median Filter
    Mat median_filtered_image2[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        medianBlur(gray_image[image_index], median_filtered_image2[image_index], MEDIAN_FILTER_SIZE_2);
        // namedWindow("Median Filtered Image2", WINDOW_AUTOSIZE);
        // imshow("Median Filtered Image2", median_filtered_image2[image_index]);
        // namedWindow("Gray Image", WINDOW_AUTOSIZE);
        // imshow("Gray Image", gray_image[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // thresholding
    Mat otsu_binary_images[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        adaptiveThreshold(median_filtered_image2[image_index], otsu_binary_images[image_index], 255, ADAPTIVE_THRESH_GAUSSIAN_C,
                          THRESH_BINARY, ADAPTIVE_THRESH_KERNEL_SIZE, 1);
        // namedWindow("Otsu Binary Image", WINDOW_AUTOSIZE);
        // imshow("Otsu Binary Image", otsu_binary_images[image_index]);
        // namedWindow("Gray Image", WINDOW_AUTOSIZE);
        // imshow("Gray Image", gray_image[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Apply Opening to remove noise
    Mat opening_images[NUM_IMAGES];
    Mat Matrix_element(OPENING_SIZE, OPENING_SIZE, CV_8U, Scalar(1));
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        morphologyEx(otsu_binary_images[image_index], opening_images[image_index], MORPH_OPEN, Matrix_element);
        // namedWindow("opening Image", WINDOW_AUTOSIZE);
        // imshow("opening Image", opening_images[image_index]);
        // namedWindow("Otsu Binary Image", WINDOW_AUTOSIZE);
        // imshow("Otsu Binary Image", otsu_binary_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // CCA
    Mat cca_images[NUM_IMAGES];
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        cca_images[image_index] = Mat::zeros(opening_images[image_index].size(), CV_8UC3);
    }
    vector<vector<vector<Point>>> contours(NUM_IMAGES);
    vector<vector<Vec4i>> hierarchy(NUM_IMAGES);
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        findContours(opening_images[image_index], contours[image_index], hierarchy[image_index], RETR_TREE, CHAIN_APPROX_NONE);
        for (int i = 0; i < contours[image_index].size(); i++)
        {
            if (contourArea(contours[image_index][i]) < MIN_AREA)
            {
                continue;
            }
            int rand1 = rand() & 0xFF;
            int rand2 = rand() & 0xFF;
            int rand3 = rand() & 0xFF;
            Scalar colour(rand1, rand2, rand3);
            drawContours(cca_images[image_index], contours[image_index], i, colour, FILLED, 8, hierarchy[image_index]);
        }
        // namedWindow("CCA Image", WINDOW_AUTOSIZE);
        // imshow("CCA Image", cca_images[image_index]);
        // namedWindow("Opening Image", WINDOW_AUTOSIZE);
        // imshow("Opening Image", opening_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Draw Bounding Boxes
    Mat bounding_box_images[NUM_IMAGES];
    vector<vector<double>> rectangularity(NUM_IMAGES);
    vector<vector<double>> circularity(NUM_IMAGES);
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        bounding_box_images[image_index] = cca_images[image_index].clone();
    }
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        vector<RotatedRect> boundRect(contours[image_index].size());
        for (int i = 0; i < contours[image_index].size(); i++)
        {
            if (contourArea(contours[image_index][i]) < MIN_AREA)
            {
                boundRect[i] = cv::RotatedRect(Point2f(0, 0), Size2f(0, 0), 0);
                rectangularity[image_index].push_back(0.01);
                circularity[image_index].push_back(0.01);
                continue;
            }
            boundRect[i] = minAreaRect(contours[image_index][i]);
            // Calculate the rectangularity of the bounding box
            rectangularity[image_index].push_back((contourArea(contours[image_index][i]) / (boundRect[i].size.width * boundRect[i].size.height)));
            circularity[image_index].push_back((4.0 * CV_PI * contourArea(contours[image_index][i])) / (pow(arcLength(contours
                                                                                                                          [image_index][i],
                                                                                                                      true),
                                                                                                            2)));
            if (rectangularity[image_index][rectangularity[image_index].size() - 1] > RECTANGULARITY_THRESHOLD && circularity
                                                                                                                          [image_index][circularity[image_index].size() - 1] < CIRCULARITY_THRESHOLD)
            {
                Scalar colour(0, 255, 0);
                Point2f rect_points[4];
                boundRect[i].points(rect_points);
                for (int j = 0; j < 4; j++)
                {
                    line(bounding_box_images[image_index], rect_points[j], rect_points[(j + 1) % 4], colour, 2, 8);
                }
            }
            else
            {
                Scalar colour(0, 0, 255);
                Point2f rect_points[4];
                boundRect[i].points(rect_points);
                for (int j = 0; j < 4; j++)
                {
                    line(bounding_box_images[image_index], rect_points[j], rect_points[(j + 1) % 4], colour, 2, 8);
                }
            }
        }
        // namedWindow("Bounding Box Image " + to_string(image_index), WINDOW_AUTOSIZE);
        // imshow("Bounding Box Image " + to_string(image_index), bounding_box_images[image_index]);
        // namedWindow("CCA Image " + to_string(image_index), WINDOW_AUTOSIZE);
        // imshow("CCA Image " + to_string(image_index), cca_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Combine the data from the Rectangularity and the large regions
    Mat combined_images[NUM_IMAGES];
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        combined_images[image_index] = Mat::zeros(cca_images[image_index].size(), CV_8UC3);
    }
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        for (int i = 0; i < contours[image_index].size(); i++)
        {
            if (contourArea(contours[image_index][i]) < MIN_AREA)
            {
                continue;
            }
            if (rectangularity[image_index][i] > RECTANGULARITY_THRESHOLD && circularity[image_index][i] < CIRCULARITY_THRESHOLD)
            {
                int rand1 = rand() & 0xFF;
                int rand2 = rand() & 0xFF;
                int rand3 = rand() & 0xFF;
                Scalar colour(rand1, rand2, rand3);
                drawContours(combined_images[image_index], contours[image_index], i, colour, FILLED, 8, hierarchy[image_index]);
            }
        }
        // namedWindow("Combined Image " + to_string(image_index + 1), WINDOW_AUTOSIZE);
        // imshow("Combined Image " + to_string(image_index + 1), combined_images[image_index]);
        // namedWindow("Bounding Box Image " + to_string(image_index + 1), WINDOW_AUTOSIZE);
        // imshow("Bounding Box Image " + to_string(image_index + 1), bounding_box_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Robert's Edge Detection
    Mat roberts_images[NUM_IMAGES];
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        roberts_images[image_index] = Mat::zeros(cca_images[image_index].size(), CV_8UC3);
    }
    Mat kernelX = (Mat_<char>(2, 2) << 1, 0, 0, -1);
    Mat kernelY = (Mat_<char>(2, 2) << 0, 1, -1, 0);
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        Mat gradX, gradY;
        // Apply the kernels to the image
        filter2D(combined_images[image_index], gradX, CV_16S, kernelX);
        filter2D(combined_images[image_index], gradY, CV_16S, kernelY);
        // Convert gradients to absolute values
        convertScaleAbs(gradX, gradX);
        convertScaleAbs(gradY, gradY);
        // Combine the gradients
        addWeighted(gradX, 0.5, gradY, 0.5, 0, roberts_images[image_index]);
        // Display the result
        // namedWindow("Roberts Image " + to_string(image_index), WINDOW_AUTOSIZE);
        // imshow("Roberts Image " + to_string(image_index), roberts_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // greyscale
    Mat roberts_grey_images[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        cvtColor(roberts_images[image_index], roberts_grey_images[image_index], COLOR_BGR2GRAY);
        // namedWindow("Gray Roberts Image", WINDOW_AUTOSIZE);
        // imshow("Gray Roberts Image", roberts_grey_images[image_index]);
        // namedWindow("Roberts Image", WINDOW_AUTOSIZE);
        // imshow("Roberts Image", roberts_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // thresholding
    Mat roberts_binary_images[NUM_IMAGES];
    for (int image_index = 0; (image_index < NUM_IMAGES); image_index++)
    {
        threshold(roberts_grey_images[image_index], roberts_binary_images[image_index], 0, 255, THRESH_BINARY);
        // namedWindow("Roberts Binary Image", WINDOW_AUTOSIZE);
        // imshow("Roberts Binary Image", roberts_binary_images[image_index]);
        // namedWindow("Roberts Image", WINDOW_AUTOSIZE);
        // imshow("Roberts Image", roberts_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Hough Line Transform
    vector<vector<Vec2f>> lines(NUM_IMAGES);
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        HoughLines(roberts_binary_images[image_index], lines[image_index], 1, CV_PI / 200.0, HOUGH_THRESHOLD);
    }
    // sort the lines
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        sort(lines[image_index].begin(), lines[image_index].end(), [](const Vec2f &a, const Vec2f &b)
             {
if (a[1] != b[1])
return a[1] < b[1];
return a[0] < b[0]; });
    }
    // Turn the images into colour images
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        cvtColor(roberts_binary_images[image_index], roberts_binary_images[image_index], COLOR_GRAY2BGR);
    }
    // Draw the lines
    Mat hough_images[NUM_IMAGES];
    bool remove_redundant = false;
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        hough_images[image_index] = roberts_binary_images[image_index].clone();
    }
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        for (size_t i = 0; i < lines[image_index].size(); i++)
        {
            remove_redundant = false;
            float rho = lines[image_index][i][0], theta = lines[image_index][i][1];
            // Extract Horizontal Lines
            cout << "Theta: " << theta << endl;
            cout << "Rho: " << rho << endl;
            if ((theta > LOWER_RATIO) && (theta < UPPER_RATIO))
            {
                // Remove redundant lines
                for (int j = i - 1; j >= 0; j--)
                {
                    float rho_prev = lines[image_index][j][0], theta_prev = lines[image_index][j][1];
                    if (!((theta_prev > LOWER_RATIO) && (theta_prev < UPPER_RATIO)))
                    {
                        continue;
                    }
                    if ((abs(rho - rho_prev) < RHO_THRESHOLD) && (abs(theta - theta_prev) < THETA_THRESHOLD))
                    {
                        remove_redundant = true;
                        break;
                    }
                }
                if (remove_redundant)
                {
                    continue;
                }
                Point pt1, pt2;
                double a = cos(theta), b = sin(theta);
                double x0 = a * rho, y0 = b * rho;
                pt1.x = cvRound(x0 + 1000 * (-b));
                pt1.y = cvRound(y0 + 1000 * (a));
                pt2.x = cvRound(x0 - 1000 * (-b));
                pt2.y = cvRound(y0 - 1000 * (a));
                line(hough_images[image_index], pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
            }
        }
        // namedWindow("Hough Image " + to_string(image_index), WINDOW_AUTOSIZE);
        // imshow("Hough Image " + to_string(image_index), hough_images[image_index]);
        // waitKey(0);
        // destroyAllWindows();
    }
    // Add the lines to the original images
    Mat hough_original_images[NUM_IMAGES];
    vector<Mat> RedLines(NUM_IMAGES);
    bool remove_redundant2 = false;
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        hough_original_images[image_index] = original_image[image_index].clone();
        RedLines[image_index] = Mat::zeros(original_image[image_index].size(), CV_8UC3);
    }
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        for (size_t i = 0; i < lines[image_index].size(); i++)
        {
            remove_redundant2 = false;
            float rho = lines[image_index][i][0], theta = lines[image_index][i][1];
            // Extract Horizontal Lines
            if ((theta > LOWER_RATIO) && (theta < UPPER_RATIO))
            {
                // Remove redundant lines
                for (int j = i - 1; j >= 0; j--)
                {
                    float rho_prev = lines[image_index][j][0], theta_prev = lines[image_index][j][1];
                    if (!((theta_prev > LOWER_RATIO) && (theta_prev < UPPER_RATIO)))
                    {
                        continue;
                    }
                    if ((abs(rho - rho_prev) < RHO_THRESHOLD) && (abs(theta - theta_prev) < (THETA_THRESHOLD)))
                    {
                        remove_redundant2 = true;
                        break;
                    }
                }
                if (remove_redundant2)
                {
                    continue;
                }
                Point pt1, pt2;
                double a = cos(theta), b = sin(theta);
                double x0 = a * rho, y0 = b * rho;
                pt1.x = cvRound(x0 + 1000 * (-b));
                pt1.y = cvRound(y0 + 1000 * (a));
                pt2.x = cvRound(x0 - 1000 * (-b));
                pt2.y = cvRound(y0 - 1000 * (a));
                line(hough_original_images[image_index], pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
                line(RedLines[image_index], pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
            }
        }
        namedWindow("Hough Original Image " + to_string(image_index), WINDOW_AUTOSIZE);
        imshow("Hough Original Image " + to_string(image_index), hough_original_images[image_index]);
        waitKey(0);
        destroyAllWindows();
    }
    // Download the Hough Original Images
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        char filename[200];
        sprintf(filename, "Media/HoughOriginalImage%d.jpg", image_index + 10);
        imwrite(filename, hough_original_images[image_index]);
    }
    // Compare the results with the ground truth
    vector<Mat> GreenLines(NUM_IMAGES);
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        GreenLines[image_index] = Mat::zeros(original_image[image_index].size(), CV_8UC3);
    }
    int index = 10;
    for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    {
        if (index == 28)
        {
            index--;
        }
        else if (index == pedestrian_crossing_ground_truth[image_index][0])
        {
            Point top_line[2];
            Point bottom_line[2];
            top_line[0] = Point(pedestrian_crossing_ground_truth[image_index][1], pedestrian_crossing_ground_truth[image_index][2]);
            top_line[1] = Point(pedestrian_crossing_ground_truth[image_index][3], pedestrian_crossing_ground_truth[image_index][4]);
            bottom_line[0] = Point(pedestrian_crossing_ground_truth[image_index][5], pedestrian_crossing_ground_truth[image_index][6]);
            bottom_line[1] = Point(pedestrian_crossing_ground_truth[image_index][7], pedestrian_crossing_ground_truth[image_index][8]);
            line(hough_original_images[index - 10], top_line[0], top_line[1], Scalar(0, 255, 0), 1, LINE_AA);
            line(hough_original_images[index - 10], bottom_line[0], bottom_line[1], Scalar(0, 255, 0), 1, LINE_AA);
            line(GreenLines[index - 10], top_line[0], top_line[1], Scalar(0, 255, 0), 1, LINE_AA);
            line(GreenLines[index - 10], bottom_line[0], bottom_line[1], Scalar(0, 255, 0), 1, LINE_AA);
        }
        else
        {
            image_index--;
        }
        namedWindow("Hough Original Image " + to_string(index), WINDOW_AUTOSIZE);
        imshow("Hough Original Image " + to_string(index), hough_original_images[image_index]);
        waitKey(0);
        destroyAllWindows();
        index++;
    }
    // Calculate the Intersection over Union
    vector<double> Union(NUM_IMAGES);
    vector<double> Intersection(NUM_IMAGES);
    vector<double> IoU(NUM_IMAGES);
    for (int i = 0; i < NUM_IMAGES; i++)
    {
        Intersection[i] = 0;
        Union[i] = 0;
        for (int j = 0; j < original_image[i].rows; j++)
        {
            for (int k = 0; k < original_image[i].cols; k++)
            {
                if ((RedLines[i].at<Vec3b>(j, k)[2] > 3) && (GreenLines[i].at<Vec3b>(j, k)[1] > 3))
                {
                    Intersection[i]++;
                    Union[i]++;
                }
                else if ((RedLines[i].at<Vec3b>(j, k)[2] > 3) || (GreenLines[i].at<Vec3b>(j, k)[1] > 3))
                {
                    Union[i]++;
                }
            }
        }
        IoU[i] = Intersection[i] / Union[i];
    }
    for (int i = 0; i < NUM_IMAGES; i++)
    {
        cout << "Intersection: " << Intersection[i] << endl;
        cout << "Union: " << Union[i] << endl;
        cout << "IoU: " << IoU[i] << endl;
    }
    // Download the Ground Truth Images
    // for (int image_index = 0; image_index < NUM_IMAGES; image_index++)
    // {
    // char filename[200];
    // sprintf(filename, "Media/GroundTruthImage%d.jpg", image_index + 10);
    // imwrite(filename, hough_original_images[image_index]);
    // }
}