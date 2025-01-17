//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Authors: Guilbert Pierre (Kitware SAS)
//          Laurenson Nick (Kitware SAS)
// Creation date: 2018-03-27
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#ifndef VTK_SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H
#define VTK_SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H

#include <LidarSlam/SpinningSensorKeypointExtractor.h>
#include <LidarSlam/DenseSpinningSensorKeypointExtractor.h>
#include <vtkObject.h>

//
// Set built-in type.  Creates member Set"name"() (e.g., SetVisibility());
//
#undef vtkCustomSetMacro
#define vtkCustomSetMacro(name, type)                                                            \
virtual void Set##name(type _arg)                                                                \
{                                                                                                \
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting " #name " to " << _arg);  \
  if (this->Extractor->Get##name() != _arg)                                                      \
  {                                                                                              \
    this->Extractor->Set##name(_arg);                                                            \
    this->Modified();                                                                            \
  }                                                                                              \
}

/**
 * @brief The class is a paraview wrapper for SpinningSensorKeypointExtractor in order to enable the
 *  creation of a proxy. This way we get a free GUI.
 *  It should only implement setter for the proxy, and a getter to the underlying keypointExtractor.
 */
class vtkSpinningSensorKeypointExtractor : public vtkObject
{
public:
  static vtkSpinningSensorKeypointExtractor* New();
  vtkTypeMacro(vtkSpinningSensorKeypointExtractor, vtkObject)
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkCustomSetMacro(NbThreads, int)

  vtkCustomSetMacro(MaxPoints, int)

  vtkCustomSetMacro(VoxelResolution, float)

  vtkCustomSetMacro(InputSamplingRatio, float)

  vtkCustomSetMacro(MinNeighNb, int)

  vtkCustomSetMacro(MinNeighRadius, float)

  vtkCustomSetMacro(MinDistanceToSensor, float)

  vtkCustomSetMacro(MaxDistanceToSensor, float)

  vtkCustomSetMacro(AzimuthMin, float)

  vtkCustomSetMacro(AzimuthMax, float)

  vtkCustomSetMacro(MinBeamSurfaceAngle, float)

  vtkCustomSetMacro(PlaneAngleThreshold, float)

  vtkCustomSetMacro(EdgeAngleThreshold, float)

  vtkCustomSetMacro(EdgeDepthGapThreshold, float)

  vtkCustomSetMacro(EdgeIntensityGapThreshold, float)

  vtkCustomSetMacro(EdgeNbGapPoints, int)

  void SetMode(int mode);

  std::shared_ptr<LidarSlam::KeypointExtractor> GetExtractor() const { return Extractor; }

protected:
  vtkSpinningSensorKeypointExtractor();

  std::shared_ptr<LidarSlam::KeypointExtractor> Extractor;

private:
  vtkSpinningSensorKeypointExtractor(const vtkSpinningSensorKeypointExtractor&) = delete;
  void operator=(const vtkSpinningSensorKeypointExtractor&) = delete;

  LidarSlam::KeypointExtractorMode Mode = LidarSlam::KeypointExtractorMode::SPARSE;
};

#endif // VTK_SPINNING_SENSOR_KEYPOINT_EXTRACTOR_H
