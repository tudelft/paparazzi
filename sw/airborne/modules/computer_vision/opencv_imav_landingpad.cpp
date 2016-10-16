/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_IMAV_landingpad.cpp"
 * @author J. Lee
 * Module for detecting landing pad in IMAV 2016
 */

#include "opencv_imav_landingpad.h"
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

// Predicate for sort
bool lexico_compare(const cv::Point2f& p1, const cv::Point2f& p2) {
    if(p1.x < p2.x) { return true; }
    if(p1.x > p2.x) { return false; }
    return (p1.y < p2.y);
}


// Predicate for unique
int CENTROID_THRESH = 10; //10 pixels away from centroid is considered duplicate
bool points_are_equal(const cv::Point2f& p1, const cv::Point2f& p2) {
//    return ((p1.x == p2.x) && (p1.y == p2.y));
    return ((p1.x < p2.x+CENTROID_THRESH) && (p1.x > p2.x-CENTROID_THRESH) && (p1.y < p2.y+CENTROID_THRESH) && (p1.y > p2.y-CENTROID_THRESH));
}

// Output
struct results landing;

float detector2_fps = 30; // initial estimate of fps
float detector2_fps_epsilon = 0.2; // used to smoothen fps

// Z-score for squares
int ZSCORE = 2;
int AREA_THRESH = 100;

struct results opencv_imav_landing(char *img, int width, int height, int v_squares, int binary_threshold, int mod, int dt)
{
    Mat M(height, width, CV_8UC2, img);
    Mat image;
    Mat binim;
    Mat imcopy;

    // Grayscale image
    cvtColor(M, image, CV_YUV2GRAY_Y422);

    // copy image
    if (mod == 1) { cvtColor(M, imcopy, CV_YUV2GRAY_Y422); }

    // Gaussian Blur
    blur(image, image, Size(5,5));

    // convert to binary image
    threshold(image, binim, binary_threshold, 255, THRESH_BINARY);

    // Canny edges
    Canny(binim, image, 66, 133);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Find contours
    findContours(image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );

    vector<vector<Point> >hull(contours.size());
    vector<vector<Point> >approx(contours.size());
    int detected_squares = 0;
    vector<Moments> mu(contours.size() );
    vector<Point2f> mc( contours.size() );

    // For contours that are above a certain size, check if it has four sides(square), then obtain centroids
    for( int i = 0; (unsigned)i < contours.size(); i++ )
    {
        double Area = contourArea(contours[i]);
        if (Area > AREA_THRESH)
        {
            convexHull(Mat(contours[i]), hull[i], false );
            approxPolyDP(Mat(hull[i]), approx[i], arcLength(Mat(hull[i]), true)*0.12, true);
            int sides = approx[i].size();
            if (sides  == 4)
            {
                mu[i] = moments(contours[i], false);
                mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
//                if (mod) {drawContours( imcopy, approx, i, 20 , 2, 8, vector<cv::Vec4i>(), 0, cv::Point() ); }
            }
        }
    }

    // Sort detected centroids
    sort(mc.begin(), mc.end(), lexico_compare);
    // Delete repeated centroids
    mc.erase(unique(mc.begin(), mc.end(), points_are_equal), mc.end());

    vector<float> centroidsx;
    vector<float> centroidsy;

    for( int i = 0; i < mc.size(); i++ )
    {
        if (mc[i].x != 0 && mc[i].y != 0)
        {
            detected_squares += 1;
            centroidsx.push_back (mc[i].x);
            centroidsy.push_back (mc[i].y);
        }
    }


    float sumx = 0;
    float sumy = 0;
    float avgx = 0;
    float avgy = 0;

    // Find average position of detected centroids
    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        sumx += centroidsx[i];
        sumy += centroidsy[i];
        if (mod) { circle(imcopy, Point(centroidsx[i],centroidsy[i]), 5, 50, 2); }
    }
    avgx = sumx / detected_squares;
    avgy = sumy / detected_squares;


    float innerx = 0;
    float innery = 0;
    float stdx = 0;
    float stdy = 0;

    // Find standard deviation of detected centroids
    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        innerx += pow((centroidsx[i] - avgx),2);
        innery += pow((centroidsy[i] - avgy),2);
    }

    stdx = sqrt(innerx/detected_squares);
    stdy = sqrt(innery/detected_squares);


    vector<float> filt_centroidx;
    vector<float> filt_centroidy;
    vector<float> ffilt_centroidx;
    vector<float> ffilt_centroidy;
    float zx;
    float zy;
    vector<float> filt_zx;
    vector<float> ffilt_zx;
    vector<float> ffilt_zy;

    // Filter out outlier centroids using z-score
    for(int i = 0; (unsigned)i < centroidsx.size(); i++)
    {
        zx = fabs((centroidsx[i]-avgx)/stdx);
        if (zx <= ZSCORE)
        {
            filt_centroidx.push_back (centroidsx[i]);
            filt_centroidy.push_back (centroidsy[i]);
            filt_zx.push_back (zx);
        }
    }


    int n_filtcentroids = 0;
    for(int i = 0; (unsigned)i < filt_centroidy.size(); i++)
    {
        zy = fabs((filt_centroidy[i]-avgy)/stdy);
        if (zy <= ZSCORE)
        {
            n_filtcentroids += 1;
            ffilt_centroidx.push_back (filt_centroidx[i]);
            ffilt_centroidy.push_back (filt_centroidy[i]);
            ffilt_zx.push_back (filt_zx[i]);
            ffilt_zy.push_back (zy);
        }
    }

    int xsum = 0;
    int ysum = 0;
    int xbar = 0;
    int ybar = 0;

    // Find the mean of filtered centroids and assume it to be the center of the marker
    // We only use this for the case when we have 2 detected squares
    for(int i = 0; (unsigned)i < ffilt_centroidx.size(); i++)
    {
        xsum += ffilt_centroidx[i];
        ysum += ffilt_centroidy[i];
    }

    //calculate distance to median
    float fmedx = 0;
    float fmedy = 0;

    // Convert separate centroids to Point
    vector<Point2f> ffilt_centroids(ffilt_centroidx.size());

    for(int i = 0; i < ffilt_centroidx.size(); i++)
    {
        ffilt_centroids[i] = Point2f (ffilt_centroidx[i], ffilt_centroidy[i]);
    }

    // sort
    sort(ffilt_centroids.begin(), ffilt_centroids.end(), lexico_compare);


    vector<float> fffilt_centroidx;
    vector<float> fffilt_centroidy;

    for( int i = 0; i < ffilt_centroids.size(); i++ )
    {
        fffilt_centroidx.push_back (ffilt_centroids[i].x);
        fffilt_centroidy.push_back (ffilt_centroids[i].y);
    }

    for(int i = 0; (unsigned)i < fffilt_centroidx.size(); i++)
    {
        if (fffilt_centroidx.size() % 2== 0)
        {
            fmedx = (fffilt_centroidx[fffilt_centroidx.size() / 2 - 1] + fffilt_centroidx[fffilt_centroidx.size() / 2]) / 2;
        }
        else
        {
            fmedx = fffilt_centroidx[fffilt_centroidx.size() / 2];
        }
    }

    for(int i = 0; (unsigned)i < fffilt_centroidy.size(); i++)
    {
        if (fffilt_centroidy.size() % 2== 0)
        {
            fmedy = (fffilt_centroidy[fffilt_centroidy.size() / 2 - 1] + fffilt_centroidy[fffilt_centroidy.size() / 2]) / 2;
        }
        else
        {
            fmedy = fffilt_centroidy[fffilt_centroidy.size() / 2];
        }
    }

    vector<float> distance_medx;
    vector<float> distance_medy;
    for(int i = 0; i < fffilt_centroidx.size(); i++)
    {
        distance_medx.push_back ( fffilt_centroidx[i]-fmedx );
        distance_medy.push_back ( fffilt_centroidy[i]-fmedy );
    }


    // If the number of filtered centroids are larger than the predetermined counter, landing pad detected
    // If the number of filtered centroids exceed 2, then we take the average of two centroids with smallest z-scores
    // as our final centroid

    if (n_filtcentroids >= v_squares)
    {
        landing.marker = 1;

        if (fffilt_centroidx.size() > 2)
        {
            vector<float> distance_xy;
            for(int i = 0; i < distance_medx.size(); i++)
            {
                distance_xy.push_back (sqrt( pow(distance_medx[i],2) + pow(distance_medy[i],2)));
            }
            //large number
            int first_index = 0;
            int second_index = 1;
            float first = distance_xy[first_index];
            float second = distance_xy[second_index];

            if (second < first)
            {
                float temp = first;
                first = second;
                second = temp;
                int temp_index = first_index;
                first_index = second_index;
                second_index = temp_index;
            }

            for(int i = 2; i < distance_xy.size(); i++)
            {
                if (distance_xy[i] < first)
                {
                    second = first;
                    second_index = first_index;
                    first = distance_xy[i];
                    first_index = i;
                }
                else
                {
                    if (distance_xy[i] < second)
                    {
                        second = distance_xy[i];
                        second_index = i;
                    }
                }
            }


            if (mod) { circle(imcopy, Point(fffilt_centroidx[first_index],fffilt_centroidy[first_index]), 20, 20, 5); }
            if (mod) { circle(imcopy, Point(fffilt_centroidx[second_index],fffilt_centroidy[second_index]), 20, 20, 5); }

            xbar = (fffilt_centroidx[first_index]+fffilt_centroidx[second_index])/2;
            ybar = (fffilt_centroidy[first_index]+fffilt_centroidy[second_index])/2;
        }
        else
        {
            xbar = (xsum / n_filtcentroids);
            ybar = (ysum / n_filtcentroids);
        }

        landing.maxx = xbar;
        landing.maxy = ybar;
        if (mod) { circle(imcopy, Point(xbar,ybar), 20, 0, 5); }
    } else
    {
        landing.marker = 0;
        landing.maxx   = 0;
        landing.maxy   = 0;
    }

    // Update FPS estimate
    detector2_fps = (1 - detector2_fps_epsilon) * detector2_fps + detector2_fps_epsilon * (1000000.f / dt);

    // Draw FPS on image
    if (mod == 1) {
        char text[50];
        sprintf(text, "FPS: %0.2f", detector2_fps);
        putText(imcopy, text, Point(10, image.rows - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255), 2);

        grayscale_opencv_to_yuv422(imcopy, img, width, height);
    }

    return landing;
}
