/**
 *  @file rgbd_keyframe.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RGBDTOOLS_RGBD_KEYFRAME_H
#define RGBDTOOLS_RGBD_KEYFRAME_H

#include <boost/filesystem.hpp>

#include "rgbdtools/rgbd_frame.h"

namespace rgbdtools {

/** @brief Extension of an RGBDFrame, which has a pose, and a 3D point cloud
 * 
 * The class is used for keyframe-based graph alignment, as well as dense
 * mapping.
 */
  
class RGBDKeyframe: public RGBDFrame
{
  public:
   
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Default (empty) constructor.
     */
    RGBDKeyframe();
  
    /** @brief Copy constructor from a RGBDFrame
     * @param frame reference to the frame which is being used to create a keyframe
     */
    RGBDKeyframe(const RGBDFrame& frame);
     
    Pose pose; ///< pose of the camera, in some fixed frame
    
    bool manually_added; ///< Whether the frame was added manually by the user

    /** @brief path length, in meters, of the camera trajectory at the moment 
     * of adding the keyframe
     */ 
    double path_length_linear; 
    
    /** @brief path length, in radians, of the camera trajectory at the 
     * moment of adding the keyframe
     */ 
    double path_length_angular;
   
    /** @brief Saves the RGBD keyframe to disk. 
    * 
    * Saves the RGBDFrame, as well as the additional keyframe info as .yml
    * 
    * @param keyframe Reference to the keyframe being saved
    * @param path The path to the folder where everything will be stored
    *  
    * @retval true  Successfully saved the data
    * @retval false Saving failed - for example, cannot create directory
    */
    static bool save(const RGBDKeyframe& keyframe, 
                     const std::string& path);
    
    /** @brief Loads the RGBD keyframe to disk. 
    * 
    * Loads the RGBDFrame, as well as the additional keyframe info as .yml
    * 
    * @param keyframe Reference to the keyframe being saved
    * @param path The path to the folder where everything will be stored
    *  
    * @retval true  Successfully loaded the data
    * @retval false Loading failed - for example, directory not found
    */
    static bool load(RGBDKeyframe& keyframe, 
                     const std::string& path);

    void storeFilteredPointCloud(double max_range, double max_stdev,double leaf_size);

    PointCloudT filteredCloud;
};

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

/** @brief Saves a vector of RGBD keyframes to disk. 
* 
* @param keyframes Reference to the keyframe being saved
* @param path The path to the folder where everything will be stored
*  
* @retval true  Successfully saved the data
* @retval false Saving failed - for example, cannot create directory
*/
bool saveKeyframes(const KeyframeVector& keyframes, 
                   const std::string& path);

/** @brief Loads a vector of RGBD keyframes to disk. 
*  
* @param keyframes Reference to the keyframe being saved
* @param path The path to the folder where everything will be stored
*  
* @retval true  Successfully saved the data
* @retval false Saving failed - for example, cannot create directory
*/
bool loadKeyframes(KeyframeVector& keyframes, 
                   const std::string& path);

} // namespace rgbdtools

#endif // RGBDTOOLS_RGBD_KEYFRAME_H
