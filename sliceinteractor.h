#ifndef SLICEINTERACTOR_H
#define SLICEINTERACTOR_H

/**
Quick and dirty visualizer for 3d vtkImageData structures. Will open a new
vtk window. To interact, hold Mouse1 and move up and down. Set to display
transversal cuts, but can be setup for sagittal or coronal planes. Normal
program execution halts until the window is closed.

Usage:

1) Include sliceinteractor.cpp into your file
2) Anywhere in the code, use a line like this:
      SliceViewer view = SliceViewer(some_image_data);
   or this
      SliceViewer view = SliceViewer(some_filter->GetOutput());
3) Close the window when done; Program resumes execution after window is closed
*/

#include "vtkSmartPointer.h"
#include "vtkImageReader2.h"
#include "vtkMatrix4x4.h"
#include "vtkImageReslice.h"
#include "vtkLookupTable.h"
#include "vtkImageMapToColors.h"
#include "vtkImageMapper.h"
#include "vtkActor2D.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkCommand.h"
#include "vtkImageData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformation.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkVertexGlyphFilter.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "vtkGlyph3D.h"
#include "vtkArrowSource.h"

#define VTK_NEW(type, instance) \
  ;                             \
  vtkSmartPointer<type> instance = vtkSmartPointer<type>::New();

// The mouse motion callback, to turn "Slicing" on and off
class vtkImageInteractionCallback : public vtkCommand {
 public:
  static vtkImageInteractionCallback *New() {
    return new vtkImageInteractionCallback;
  };

  vtkImageInteractionCallback() {
    this->Slicing = 0;
    this->ImageReslice = 0;
    this->Interactor = 0;
  };

  void SetImageReslice(vtkImageReslice *reslice) {
    this->ImageReslice = reslice;
  };

  vtkImageReslice *GetImageReslice() {
    return this->ImageReslice;
  };

  void SetInteractor(vtkRenderWindowInteractor *interactor) {
    this->Interactor = interactor;
  };

  vtkRenderWindowInteractor *GetInteractor() {
    return this->Interactor;
  };

  virtual void Execute(vtkObject *, unsigned long event, void *) {
    vtkRenderWindowInteractor *interactor = this->GetInteractor();

    int lastPos[2];
    interactor->GetLastEventPosition(lastPos);
    int currPos[2];
    interactor->GetEventPosition(currPos);

    if (event == vtkCommand::LeftButtonPressEvent) {
      this->Slicing = 1;
    } else if (event == vtkCommand::LeftButtonReleaseEvent) {
      this->Slicing = 0;
    } else if (event == vtkCommand::MouseMoveEvent) {
      if (this->Slicing) {
        vtkImageReslice *reslice = this->ImageReslice;

        // Increment slice position by deltaY of mouse
        int deltaY = lastPos[1] - currPos[1];

        reslice->Update();
        double sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
        vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
        // move the center point that we are slicing through
        double point[4];
        double center[4];
        point[0] = sliceSpacing * deltaY;
        point[1] = sliceSpacing * deltaY;
        point[2] = sliceSpacing * deltaY;
        point[3] = 1.0;
        matrix->MultiplyPoint(point, center);
        matrix->SetElement(0, 3, center[0]);
        matrix->SetElement(1, 3, center[1]);
        matrix->SetElement(2, 3, center[2]);
        interactor->Render();
      } else {
        vtkInteractorStyle *style =
            vtkInteractorStyle::SafeDownCast(interactor->GetInteractorStyle());
        if (style) {
          style->OnMouseMove();
        }
      }
    }
  };

 private:
  // Actions (slicing only, for now)
  int Slicing;

  // Pointer to vtkImageReslice
  vtkImageReslice *ImageReslice;

  // Pointer to the interactor
  vtkRenderWindowInteractor *Interactor;
};

class SliceViewer {
 public: 

  static void viewPolyData(vtkPolyData* polydata)
  {
    VTK_NEW(vtkPoints, pts);
    pts->DeepCopy(polydata->GetPoints());

    VTK_NEW(vtkPolyData, new_polydata);
    new_polydata->SetPoints(pts);

    VTK_NEW(vtkVertexGlyphFilter, vert_filter);
    vert_filter->SetInput(new_polydata);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(vert_filter->GetOutputPort());

    VTK_NEW(vtkActor, actor);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.8, 0.8);

    VTK_NEW(vtkRenderer, renderer);
    renderer->AddActor(actor);

    VTK_NEW(vtkRenderWindowInteractor, interactor);

    VTK_NEW(vtkRenderWindow, window);
    window->AddRenderer(renderer);
    window->SetInteractor(interactor);
    window->SetSize(1024, 768);

    renderer->Render();
    interactor->Start(); //Will not return until window is closed
  }

  static void viewPolyDataWithNormals(vtkPolyData* polydata)
  {
    //Polydata
    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInput(polydata);
    mapper->SetScalarVisibility(0);

    VTK_NEW(vtkActor, actor);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.8, 0.8);


    //Arrows
    VTK_NEW(vtkArrowSource, arrow);

    VTK_NEW(vtkGlyph3D, glyph);
    glyph->SetInput(polydata);
    glyph->SetSource(arrow->GetOutput());
    glyph->SetVectorModeToUseNormal();
    glyph->SetScaleFactor(0.05);

    VTK_NEW(vtkPolyDataMapper, arrow_mapper);
    arrow_mapper->SetInputConnection(glyph->GetOutputPort());

    VTK_NEW(vtkActor, arrow_actor);
    arrow_actor->SetMapper(arrow_mapper);
    arrow_actor->GetProperty()->SetColor(0.8, 0.8, 0.2);


    //Rendering
    VTK_NEW(vtkRenderer, renderer);
    renderer->AddActor(actor);
    renderer->AddActor(arrow_actor);

    VTK_NEW(vtkRenderWindowInteractor, interactor);

    VTK_NEW(vtkRenderWindow, window);
    window->AddRenderer(renderer);
    window->SetInteractor(interactor);
    window->SetSize(1024, 768);

    renderer->Render();
    interactor->Start(); //Will not return until window is closed
  }

  static void view3dImageData(vtkImageData *volume) {

    // Easiest way of preventing modifications to trigger updates
    VTK_NEW(vtkImageData, volume_copy);
    volume_copy->DeepCopy(volume);

    // Calculate the center of the volume
    int extent[6];
    double spacing[3];
    double origin[3];

    volume_copy->GetExtent(extent);
    volume_copy->GetSpacing(spacing);
    volume_copy->GetOrigin(origin);

    double center[3];
    center[0] = origin[0] + spacing[0] * 0.5 * (extent[0] + extent[1]);
    center[1] = origin[1] + spacing[1] * 0.5 * (extent[2] + extent[3]);
    center[2] = origin[2] + spacing[2] * 0.5 * (extent[4] + extent[5]);

    // Matrices for axial, coronal, sagittal, oblique view orientations
    static double axialElements[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};

//    static double coronalElements[16] = {1, 0,  0, 0, 0, 0, 1, 0,
//                                         0, -1, 0, 0, 0, 0, 0, 1};

//    static double sagittalElements[16] = {0, 0,  -1, 0, 1, 0, 0, 0,
//                                          0, -1, 0,  0, 0, 0, 0, 1};

    // static double obliqueElements[16] = {
    //         1, 0, 0, 0,
    //         0, 0.866025, -0.5, 0,
    //         0, 0.5, 0.866025, 0,
    //         0, 0, 0, 1 };

    // Set the slice orientation
    vtkSmartPointer<vtkMatrix4x4> resliceAxes =
        vtkSmartPointer<vtkMatrix4x4>::New();
    resliceAxes->DeepCopy(axialElements);
    // Set the point through which to slice
    resliceAxes->SetElement(0, 3, center[0]);
    resliceAxes->SetElement(1, 3, center[1]);
    resliceAxes->SetElement(2, 3, center[2]);

    // Extract a slice in the desired orientation
    vtkSmartPointer<vtkImageReslice> reslice =
        vtkSmartPointer<vtkImageReslice>::New();
    reslice->SetInput(volume_copy);
    reslice->SetOutputDimensionality(2);
    reslice->SetResliceAxes(resliceAxes);
    reslice->SetInterpolationModeToLinear();

    // Display the image
    vtkSmartPointer<vtkImageMapper> image_mapper =
        vtkSmartPointer<vtkImageMapper>::New();
    image_mapper->SetInputConnection(reslice->GetOutputPort());
    image_mapper->SetColorWindow(255.0);
    image_mapper->SetColorLevel(127.5);

    vtkSmartPointer<vtkActor2D> actor = vtkSmartPointer<vtkActor2D>::New();
    actor->SetMapper(image_mapper);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);

    vtkSmartPointer<vtkRenderWindow> window =
        vtkSmartPointer<vtkRenderWindow>::New();
    window->AddRenderer(renderer);

    // Set up the interaction
    vtkSmartPointer<vtkInteractorStyleImage> imageStyle =
        vtkSmartPointer<vtkInteractorStyleImage>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> interactor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetInteractorStyle(imageStyle);
    window->SetInteractor(interactor);
    window->Render();

    vtkSmartPointer<vtkImageInteractionCallback> callback =
        vtkSmartPointer<vtkImageInteractionCallback>::New();
    callback->SetImageReslice(reslice);
    callback->SetInteractor(interactor);

    imageStyle->AddObserver(vtkCommand::MouseMoveEvent, callback);
    imageStyle->AddObserver(vtkCommand::LeftButtonPressEvent, callback);
    imageStyle->AddObserver(vtkCommand::LeftButtonReleaseEvent, callback);

    // Start interaction
    // The Start() method doesn't return until the window is closed by the user
    interactor->Start();
  }

  private:
  SliceViewer();
};

#endif  // SLICEINTERACTOR_H
