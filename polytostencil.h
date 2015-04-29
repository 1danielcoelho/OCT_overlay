// The current version of VTK (5.8) has a very outdated
// vtkPolyDataToImageStencil
// filter, which produces undesirable artifacts on its outputs. The
// PolyToStencil
// class is a container to hold the algorithm of the newer version, along with
// the simplest form of interface (that simulates a VTK interface). The
// following
// license boilerplate was present on the file this algorithm was extracted
// from.

/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPolyDataToImageStencil.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*=========================================================================

Copyright (c) 2008 Atamai, Inc.

Use, modification and redistribution of the software, in source or
binary forms, are permitted provided that the following terms and
conditions are met:

1) Redistribution of the source code, in verbatim or modified
   form, must retain the above copyright notice, this license,
   the following disclaimer, and any notices that refer to this
   license and/or the following disclaimer.

2) Redistribution in binary form must include the above copyright
   notice, a copy of this license and the following disclaimer
   in the documentation or with other materials provided with the
   distribution.

3) Modified copies of the source code must be clearly marked as such,
   and must not be misrepresented as verbatim copies of the source code.

THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE SOFTWARE "AS IS"
WITHOUT EXPRESSED OR IMPLIED WARRANTY INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  IN NO EVENT SHALL ANY COPYRIGHT HOLDER OR OTHER PARTY WHO MAY
MODIFY AND/OR REDISTRIBUTE THE SOFTWARE UNDER THE TERMS OF THIS LICENSE
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, LOSS OF DATA OR DATA BECOMING INACCURATE
OR LOSS OF PROFIT OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF
THE USE OR INABILITY TO USE THE SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.

=========================================================================*/

#ifndef POLYTOSTENCIL_H
#define POLYTOSTENCIL_H

#include "vtkSmartPointer.h"

#include "vtkImageStencilData.h"
#include "vtkObjectFactory.h"
#include "vtkMath.h"
#include "vtkCellArray.h"
#include "vtkDoubleArray.h"
#include "vtkSignedCharArray.h"
#include "vtkPoints.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkGenericCell.h"
#include "vtkImageData.h"
#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkImageStencilSource.h"

#include <map>
#include <vector>
#include <utility>
#include <algorithm>

#include <math.h>

#include "defines.h"

class PolyToStencil {
 public:
  PolyToStencil();
  ~PolyToStencil();

  void SetInput(vtkPolyData *input);
  vtkPolyData* GetInput();

  void SetTolerance(double tolerance);
  double GetTolerance();

  void SetInformationInput(vtkImageData *info_input);
  void Update();

  vtkImageStencilData *GetOutput();

  void ThreadedExecute();
  void PolyDataCutter(vtkPolyData *input, vtkPolyData *output, double z);
  void PolyDataSelector(vtkPolyData *input, vtkPolyData *output, double z,
                        double thickness);

 private:
  double m_tolerance;

  vtkSmartPointer<vtkPolyData> m_input;
  vtkSmartPointer<vtkImageStencilData> m_output;
};

#endif  // POLYTOSTENCIL_H
