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

#ifndef NRSLAM_CIRCLEFILTER_H
#define NRSLAM_CIRCLEFILTER_H

#include "masking/filter.h"

/*
 * This class defines a mask over a circle inscribed in the image. It masks out:
 * - cx and cy offset of the center of the circle from the center of the image (in image coordinates)
 */

class CircleFilter : public Filter
{
public:
    CircleFilter(int cx, int cy) : cx_(cx), cy_(cy)
    {
        filter_name_ = "CircleFilter";
    }

    cv::Mat generateMask(const cv::Mat &im);

    std::string getDescription();

private:
    int cx_, cy_;
};

#endif // NRSLAM_CIRCLEFILTER_H
