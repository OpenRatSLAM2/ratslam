/*
 * openRatSLAM
 *
 * posecell_network - Implements the continuous attractor network (pose cells) for RatSLAM.
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

/**
 * @file posecell_network.h
 * @brief Defines the PosecellNetwork class for RatSLAM pose cell dynamics.
 */

#ifndef _POSE_CELL_NETWORK_HPP
#define _POSE_CELL_NETWORK_HPP

#define _USE_MATH_DEFINES
#include <stdio.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>

#include "math.h"

typedef double Posecell;

namespace ratslam
{

/**
 * @struct PosecellVisualTemplate
 * @brief Stores information about a visual template in the pose cell network.
 */
  struct PosecellVisualTemplate
  {
    unsigned int id;               ///< Template ID
    double pc_x, pc_y, pc_th;      ///< Pose cell coordinates
    double decay;                  ///< Decay value for template
    std::vector < unsigned int > exps; ///< Associated experiences

    template < typename Archive >
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & id;
      ar & pc_x;
      ar & pc_y;
      ar & pc_th;
      ar & decay;
      ar & exps;
    }
  };

/**
 * @struct PosecellExperience
 * @brief Represents a node (experience) in the pose cell experience map.
 */
  struct PosecellExperience
  {
    double x_pc, y_pc, th_pc; ///< Pose cell coordinates
    int vt_id;               ///< Associated visual template ID
  };

  class PosecellNetwork
  {
public:
    friend class PosecellScene;

    enum PosecellAction { NO_ACTION = 0, CREATE_NODE, CREATE_EDGE, SET_NODE };

    PosecellNetwork(
      int pc_dim_xy, int pc_dim_th, int pc_w_e_dim, int pc_w_i_dim, double pc_w_e_var,
      double pc_w_i_var, double pc_global_inhib, double vt_active_decay, double pc_vt_inject_energy,
      double pc_cell_x_size, double exp_delta_pc_threshold, double pc_vt_restore);
    ~PosecellNetwork();

    void on_odo(double vtrans, double vrot, double time_diff_s);

    void on_view_template(unsigned int vt, double vt_rad);

    PosecellAction get_action();

    // these updated by find_best()
    double x() {return best_x;}
    double y() {return best_y;}
    double th() {return best_th;}

    // get and set all the cells as one array
    double * get_cells();
    bool set_cells(double * cells);

    // access to some of the constants specified in
    // RatSLAM properties.
    double get_delta_pc(double x, double y, double th);

    unsigned int get_current_exp_id() {return current_exp;}

    double get_relative_rad() {return vt_delta_pc_th * 2.0 * M_PI / PC_DIM_TH;}

    template < typename Archive >
    void save(Archive & ar, const unsigned int version) const
    {
      ar & PC_DIM_XY;
      ar & PC_DIM_TH;
      ar & PC_W_E_DIM;
      ar & PC_W_I_DIM;
      ar & PC_W_E_VAR;
      ar & PC_W_I_VAR;
      ar & PC_GLOBAL_INHIB;

      ar & VT_ACTIVE_DECAY;
      ar & PC_VT_RESTORE;

      ar & best_x;
      ar & best_y;
      ar & best_th;

      int i, j, k;
      for (k = 0; k < PC_DIM_TH; k++) {
        for (j = 0; j < PC_DIM_XY; j++) {
          for (i = 0; i < PC_DIM_XY; i++) {
            ar & posecells[k][j][i];
          }
        }
      }
    }

    template < typename Archive >
    void load(Archive & ar, const unsigned int version)
    {
      ar & PC_DIM_XY;
      ar & PC_DIM_TH;
      ar & PC_W_E_DIM;
      ar & PC_W_I_DIM;
      ar & PC_W_E_VAR;
      ar & PC_W_I_VAR;
      ar & PC_GLOBAL_INHIB;

      ar & VT_ACTIVE_DECAY;
      ar & PC_VT_RESTORE;

      ar & best_x;
      ar & best_y;
      ar & best_th;

      pose_cell_builder();

      int i, j, k;
      for (k = 0; k < PC_DIM_TH; k++) {
        for (j = 0; j < PC_DIM_XY; j++) {
          for (i = 0; i < PC_DIM_XY; i++) {
            ar & posecells[k][j][i];
          }
        }
      }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

private:
    friend class boost::serialization::access;

    void create_experience();
    void create_view_template();

    // inject energy into a specific point in the network
    bool inject(int act_x, int act_y, int act_z, double energy);

    // locally excite and inhibit points. Excite spreads energy and
    // inhibit compresses.
    bool excite();
    bool inhibit();

    // global inhibition
    bool global_inhibit();

    // normalise all the energy in the system
    bool normalise();

    // shift the energy in the system by a
    // translational and rotational velocity.
    bool path_integration(double vtrans, double vrot);

    // find an approximation of the centre of the energy
    // packet.
    double find_best();

    PosecellNetwork() {
    }
    PosecellNetwork(const PosecellNetwork & other);
    const PosecellNetwork & operator = (const PosecellNetwork & other);
    void pose_cell_builder();
    bool pose_cell_excite_helper(int x, int y, int z);
    bool pose_cell_inhibit_helper(int x, int y, int z);
    void circshift2d(
      double * array, double * array_buffer, int dimx, int dimy, int shiftx, int shifty);
    int rot90_square(double ** array, int dim, int rot);
    int generate_wrap(int * wrap, int start1, int end1, int start2, int end2, int start3, int end3);
    double norm2d(double var, int x, int y, int z, int dim_centre);
    double get_min_delta(double d1, double d2, double max);

    int PC_DIM_XY;
    int PC_DIM_TH;
    int PC_W_E_DIM;
    int PC_W_I_DIM;
    int PC_W_E_VAR;
    int PC_W_I_VAR;
    double PC_GLOBAL_INHIB;

    double VT_ACTIVE_DECAY;
    double PC_VT_RESTORE;
    double PC_VT_INJECT_ENERGY;
    double PC_CELL_X_SIZE;

    double EXP_DELTA_PC_THRESHOLD;

    double best_x;
    double best_y;
    double best_th;

    double vt_delta_pc_th;

    bool odo_update;
    bool vt_update;

    Posecell *** posecells;
    Posecell * posecells_memory;
    int posecells_memory_size;
    int posecells_elements;
    Posecell *** pca_new;
    Posecell * pca_new_memory;
    Posecell ** pca_new_rot_ptr;
    Posecell ** pca_new_rot_ptr2;
    Posecell * posecells_plane_th;
    double * PC_W_EXCITE;
    double * PC_W_INHIB;

    int PC_W_E_DIM_HALF;
    int PC_W_I_DIM_HALF;

    int * PC_E_XY_WRAP;
    int * PC_E_TH_WRAP;
    int * PC_I_XY_WRAP;
    int * PC_I_TH_WRAP;

    int PC_CELLS_TO_AVG;
    int * PC_AVG_XY_WRAP;
    int * PC_AVG_TH_WRAP;

    double * PC_XY_SUM_SIN_LOOKUP;
    double * PC_XY_SUM_COS_LOOKUP;
    double * PC_TH_SUM_SIN_LOOKUP;
    double * PC_TH_SUM_COS_LOOKUP;

    double PC_C_SIZE_TH;

    std::vector < PosecellVisualTemplate > visual_templates;
    std::vector < PosecellExperience > experiences;

    unsigned int current_vt, prev_vt;
    unsigned int current_exp, prev_exp;
  };

}  // namespace ratslam

#endif  // _POSE_CELL_NETWORK_HPP
