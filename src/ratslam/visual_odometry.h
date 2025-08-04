/*
 * openRatSLAM
 *
 * VisualOdometry - Visual odometry computation for RatSLAM using image intensity profiles.
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RATSLAM_VISUAL_ODOMETRY_H
#define RATSLAM_VISUAL_ODOMETRY_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace ratslam
{

/**
 * @class VisualOdometry
 * @brief Computes visual odometry from image intensity profiles for RatSLAM.
 *
 * This class processes images and computes translational and rotational odometry
 * using intensity profiles extracted from specified image regions.
 */
  class VisualOdometry
  {
public:
    /**
         * @brief Constructor for VisualOdometry.
         * @param vtrans_image_x_min Minimum X for translation region.
         * @param vtrans_image_x_max Maximum X for translation region.
         * @param vtrans_image_y_min Minimum Y for translation region.
         * @param vtrans_image_y_max Maximum Y for translation region.
         * @param vrot_image_x_min Minimum X for rotation region.
         * @param vrot_image_x_max Maximum X for rotation region.
         * @param vrot_image_y_min Minimum Y for rotation region.
         * @param vrot_image_y_max Maximum Y for rotation region.
         * @param camera_fov_deg Camera field of view in degrees.
         * @param camera_hz Camera frame rate in Hz.
         * @param vtrans_scaling Scaling factor for translation.
         * @param vtrans_max Maximum translation value.
         */
    VisualOdometry(
      int vtrans_image_x_min, int vtrans_image_x_max, int vtrans_image_y_min,
      int vtrans_image_y_max,
      int vrot_image_x_min, int vrot_image_x_max, int vrot_image_y_min, int vrot_image_y_max,
      double camera_fov_deg, double camera_hz, double vtrans_scaling, double vtrans_max);

    /**
         * @brief Process a new image and compute odometry.
         * @param data Pointer to image data.
         * @param greyscale Whether the image is greyscale.
         * @param image_width Image width in pixels.
         * @param image_height Image height in pixels.
         * @param vtrans_ms Output: computed translation (m/s).
         * @param vrot_rads Output: computed rotation (rad/s).
         */
    void on_image(
      const unsigned char * data, bool greyscale, unsigned int image_width,
      unsigned int image_height,
      double * vtrans_ms, double * vrot_rads);

    /**
         * @brief Serialization for VisualOdometry (for ROS 2 and Boost compatibility).
         */
    template < typename Archive >
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & IMAGE_HEIGHT;
      ar & IMAGE_WIDTH;
      ar & VTRANS_IMAGE_X_MIN;
      ar & VTRANS_IMAGE_X_MAX;
      ar & VTRANS_IMAGE_Y_MIN;
      ar & VTRANS_IMAGE_Y_MAX;
      ar & VROT_IMAGE_X_MIN;
      ar & VROT_IMAGE_X_MAX;
      ar & VROT_IMAGE_Y_MIN;
      ar & VROT_IMAGE_Y_MAX;
      ar & CAMERA_FOV_DEG;
      ar & CAMERA_HZ;
      ar & VTRANS_SCALING;
      ar & VTRANS_MAX;
      ar & vtrans_profile;
      ar & vrot_profile;
      ar & vtrans_prev_profile;
      ar & vrot_prev_profile;
      ar & first;
    }

private:
    VisualOdometry() {
    }

    int IMAGE_HEIGHT; ///< Image height in pixels
    int IMAGE_WIDTH; ///< Image width in pixels

    int VTRANS_IMAGE_X_MIN; ///< Min X for translation region
    int VTRANS_IMAGE_X_MAX; ///< Max X for translation region
    int VTRANS_IMAGE_Y_MIN; ///< Min Y for translation region
    int VTRANS_IMAGE_Y_MAX; ///< Max Y for translation region

    int VROT_IMAGE_X_MIN; ///< Min X for rotation region
    int VROT_IMAGE_X_MAX; ///< Max X for rotation region
    int VROT_IMAGE_Y_MIN; ///< Min Y for rotation region
    int VROT_IMAGE_Y_MAX; ///< Max Y for rotation region

    double CAMERA_FOV_DEG; ///< Camera field of view (degrees)
    double CAMERA_HZ;     ///< Camera frame rate (Hz)

    double VTRANS_SCALING; ///< Scaling factor for translation
    double VTRANS_MAX;    ///< Maximum translation value

    std::vector < double > vtrans_profile;  ///< Current translation profile
    std::vector < double > vrot_profile;    ///< Current rotation profile
    std::vector < double > vtrans_prev_profile; ///< Previous translation profile
    std::vector < double > vrot_prev_profile; ///< Previous rotation profile

    bool first; ///< True if first image processed

    /**
         * @brief Compute visual odometry from intensity profiles.
         * @param data Current intensity profile.
         * @param width Profile width.
         * @param olddata Previous intensity profile.
         * @param vtrans_ms Output: translation (m/s).
         * @param vrot_rads Output: rotation (rad/s).
         */
    void visual_odo(
      double * data, unsigned short width, double * olddata, double * vtrans_ms,
      double * vrot_rads);

    /**
         * @brief Convert an image region to a 1D intensity profile.
         * @param current_view Output: intensity profile.
         * @param view_rgb Input image data.
         * @param grayscale Whether the image is greyscale.
         * @param X_RANGE_MIN Min X for region.
         * @param X_RANGE_MAX Max X for region.
         * @param Y_RANGE_MIN Min Y for region.
         * @param Y_RANGE_MAX Max Y for region.
         */
    void convert_view_to_view_template(
      double * current_view, const unsigned char * view_rgb, bool grayscale, int X_RANGE_MIN,
      int X_RANGE_MAX, int Y_RANGE_MIN, int Y_RANGE_MAX);
  };

}  // namespace ratslam

#endif  // RATSLAM_VISUAL_ODOMETRY_H
