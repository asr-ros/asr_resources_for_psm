/**

Copyright (c) 2016, Gehrung Joachim, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "Pose.h"

namespace ResourcesForPsm {
  
  Pose::Pose()
  {
    mPosition.reset(new Eigen::Vector3d(0, 0, 0));
    mOrientation.reset(new Eigen::Quaternion<double>(1, 0, 0, 0));
  }
  
  Pose::Pose(const asr_msgs::AsrObject pObject)
  {

    if(!pObject.sampledPoses.size()){
      std::cerr << "Got a AsrObject without poses." << std::endl;
      std::exit(1);    
    }

    // Copy the position.
    mPosition.reset(new Eigen::Vector3d(pObject.sampledPoses.front().pose.position.x,
					pObject.sampledPoses.front().pose.position.y,
					pObject.sampledPoses.front().pose.position.z));
    
    // Copy the orientation.
    mOrientation.reset(new Eigen::Quaternion<double>(pObject.sampledPoses.front().pose.orientation.w,
						     pObject.sampledPoses.front().pose.orientation.x,
						     pObject.sampledPoses.front().pose.orientation.y,
						     pObject.sampledPoses.front().pose.orientation.z));

  }
  
  Pose::Pose(const geometry_msgs::Pose pPose)
  {
    // Copy the position.
    mPosition.reset(new Eigen::Vector3d(pPose.position.x,
					pPose.position.y,
					pPose.position.z));
    
    // Copy the orientation.
    mOrientation.reset(new Eigen::Quaternion<double>(pPose.orientation.w,
						     pPose.orientation.x,
						     pPose.orientation.y,
						     pPose.orientation.z));
  }
  
  Pose::Pose(const boost::shared_ptr<SceneModel::Object> pObject)
  { 
    mPosition = pObject->mPosition;
    mOrientation = pObject->mOrientation;
  }
  
  Pose::~Pose()
  {
  }
  
  void Pose::convertPoseIntoFrame(const boost::shared_ptr<Pose>& pFrame, boost::shared_ptr<Pose>& pResult)
  {
    // Initialize the result pointer.
    pResult.reset(new Pose());
    
    // Calculate the relative position by subtracting the position of the child from the parent position.
    // Rotate the resulting relative position into the parent frame.
//     pResult->mPosition.reset(new Eigen::Vector3d(pFrame->mOrientation->toRotationMatrix().inverse() * (*pFrame->mPosition - *mPosition)));
    pResult->mPosition.reset(new Eigen::Vector3d(pFrame->mOrientation->toRotationMatrix().inverse() * (*mPosition - *pFrame->mPosition)));
    
    // The relative orientation is defined as the difference between the orientation of parent and child.
    pResult->mOrientation.reset(new Eigen::Quaternion<double>(*mOrientation * pFrame->mOrientation->inverse()));
  }
  
  Eigen::Vector3d Pose::getPosition()
  {
    return *mPosition;
  }
  
  Eigen::Quaternion<double> Pose::getOrientation()
  {
    return *mOrientation;
  }
  
}
