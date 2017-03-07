/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global includes
#include <cmath>
#include <vector>

// Package includes
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include <asr_msgs/AsrObject.h>

#include <geometry_msgs/Pose.h>

#include <trainer/source/Object.h>

namespace ResourcesForPsm {
  
  /**
   * Class for storing object poses and performing pose related operations.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class Pose {
  public:
    
    /**
     * Constructor.
     */
    Pose();
    
    /**
     * Constructor.
     * 
     * @param pObject The AsrObject to acquire the pose from.
     */
    Pose(const asr_msgs::AsrObject pObject);
    
    /**
     * Constructor.
     * 
     * @param pObject The pose to convert.
     */
    Pose(const geometry_msgs::Pose pPose);
    
    /**
     * Constructor.
     * 
     * @param pObject The object to acquire the pose from.
     */
    Pose(const boost::shared_ptr<SceneModel::Object> pObject);
    
    /**
     * Destructor.
     */
    ~Pose();
    
    /**
     * Converts the pose represented by pChild into a pose in the frame represented by pParent. The result is stored in pResult.
     * 
     * @param pFrame The frame to convert the pose into.
     * @param pResult The result of the calculation.
     */
    void convertPoseIntoFrame(const boost::shared_ptr<Pose>& pFrame, boost::shared_ptr<Pose>& pResult);
    
    /**
     * Returns the position.
     */
    Eigen::Vector3d getPosition();
    
    /**
     * Returns the orientation.
     */
    Eigen::Quaternion<double> getOrientation();
    
  private:
    
    /**
     * The position part of the pose, modelled in euclidian space.
     */
    boost::shared_ptr<Eigen::Vector3d> mPosition;
    
    /**
     * The orientation part of the pose modelled as a quaternion.
     */
    boost::shared_ptr<Eigen::Quaternion<double> > mOrientation;
  };
}
