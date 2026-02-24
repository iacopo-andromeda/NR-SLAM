/*
 * This file is part of NR-SLAM
 *
 * Copyright (C) 2022-2023 Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * NR-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "circle_filter.h"

#include <opencv2/imgproc.hpp>

cv::Mat CircleFilter::generateMask(const cv::Mat &im)
{

    cv::Mat mask = cv::Mat::zeros(im.size(), CV_8U);

    auto cx = std::clamp(im.cols / 2 + cx_, 0, im.cols - 1);
    auto cy = std::clamp(im.rows / 2 + cy_, 0, im.rows - 1);

    auto r = std::min({cx, cy, im.cols - cx, im.rows - cy});

    cv::circle(mask, cv::Point(cx, cy), r, cv::Scalar(255), cv::FILLED);
    return mask;
}

std::string CircleFilter::getDescription()
{
    return std::string("Circle mask with parameters [cx=" + std::to_string(cx_) + ", cy=" + std::to_string(cy_) + "]");
}