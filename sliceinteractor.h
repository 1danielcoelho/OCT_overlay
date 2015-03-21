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
#include "vtkMarchingSquares.h"

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
    this->MarchingSquares = 0;
    this->Renderer = 0;
    this->Interactor = 0;
    this->squares_threshold = 128;
  };

  void SetRenderer(vtkRenderer *renderer) {
    this->Renderer = renderer;
  };

  vtkRenderer *GetRenderer() {
    return this->Renderer;
  };

  void SetImageReslice(vtkImageReslice *reslice) {
    this->ImageReslice = reslice;
  };

  vtkImageReslice *GetImageReslice() {
    return this->ImageReslice;
  };

  void SetMarchingSquares(vtkMarchingSquares *squares) {
    this->MarchingSquares = squares;
  };

  vtkMarchingSquares *GetMarchingSquares() {
    return this->MarchingSquares;
  };

  void SetInteractor(vtkRenderWindowInteractor *interactor) {
    this->Interactor = interactor;
  };

  vtkRenderWindowInteractor *GetInteractor() {
    return this->Interactor;
  };

  double GetThreshold() { return squares_threshold; }

  virtual void Execute(vtkObject *, unsigned long event, void *) {

    vtkRenderWindowInteractor *interactor = this->GetInteractor();

    int lastPos[2];
    interactor->GetLastEventPosition(lastPos);
    int currPos[2];
    interactor->GetEventPosition(currPos);

    if (event == vtkCommand::MouseWheelForwardEvent) {
      this->squares_threshold += 1;
      if (this->squares_threshold > 255) this->squares_threshold = 255;

      std::cout << "Threshold is now " << this->squares_threshold << std::endl;
      this->MarchingSquares->SetValue(0, this->squares_threshold);
      this->MarchingSquares->Update();
      this->Interactor->Render();
    } else if (event == vtkCommand::MouseWheelBackwardEvent) {
      this->squares_threshold -= 1;
      if (this->squares_threshold < 0) this->squares_threshold = 0;

      std::cout << "Threshold is now " << this->squares_threshold << std::endl;
      this->MarchingSquares->SetValue(0, this->squares_threshold);
      this->MarchingSquares->Update();
      this->Interactor->Render();
    } else if (event == vtkCommand::LeftButtonPressEvent) {
      this->Slicing = 1;
    } else if (event == vtkCommand::LeftButtonReleaseEvent) {
      this->Slicing = 0;
    } else if (event == vtkCommand::MouseMoveEvent) {
      if (this->Slicing) {
        vtkImageReslice *reslice = this->ImageReslice;
        vtkRenderer *renderer = this->Renderer;

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

        renderer->ResetCamera();

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
  double squares_threshold;

  // Pointer to vtkImageReslice
  vtkImageReslice *ImageReslice;
  vtkMarchingSquares *MarchingSquares;
  vtkRenderer *Renderer;

  // Pointer to the interactor
  vtkRenderWindowInteractor *Interactor;
};

class SliceViewer {
 public:
  static void viewPolyData(vtkPolyData *polydata) {
    // Simple way of dropping cell data without changing input
    VTK_NEW(vtkPoints, pts);
    pts->ShallowCopy(polydata->GetPoints());

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
    interactor->Start();  // Will not return until window is closed
  }

  static void viewPolyDataWithNormals(vtkPolyData *polydata) {
    // Polydata
    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInput(polydata);
    mapper->SetScalarVisibility(0);

    VTK_NEW(vtkActor, actor);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.8, 0.8);

    // Arrows
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

    // Rendering
    VTK_NEW(vtkRenderer, renderer);
    renderer->AddActor(actor);
    renderer->AddActor(arrow_actor);

    VTK_NEW(vtkRenderWindowInteractor, interactor);

    VTK_NEW(vtkRenderWindow, window);
    window->AddRenderer(renderer);
    window->SetInteractor(interactor);
    window->SetSize(1024, 768);

    renderer->Render();
    interactor->Start();  // Will not return until window is closed
  }

  static double view3dImageData(vtkImageData *volume) {

    // Easiest way of preventing modifications to trigger updates
    VTK_NEW(vtkImageData, volume_copy);
    volume_copy->ShallowCopy(volume);

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
    static double axial_elements[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};

    //    static double coronal_elements[16] = {1, 0,  0, 0, 0, 0, 1, 0,
    //                                         0, -1, 0, 0, 0, 0, 0, 1};

    //    static double sagittal_elements[16] = {0, 0,  -1, 0, 1, 0, 0, 0,
    //                                          0, -1, 0,  0, 0, 0, 0, 1};

    // static double oblique_elements[16] = {
    //         1, 0, 0, 0,
    //         0, 0.866025, -0.5, 0,
    //         0, 0.5, 0.866025, 0,
    //         0, 0, 0, 1 };

    // Set the slice orientation
    VTK_NEW(vtkMatrix4x4, reslice_axes);
    reslice_axes->DeepCopy(axial_elements);
    // Set the point through which to slice
    reslice_axes->SetElement(0, 3, center[0]);
    reslice_axes->SetElement(1, 3, center[1]);
    reslice_axes->SetElement(2, 3, center[2]);

    // Extract a slice in the desired orientation
    VTK_NEW(vtkImageReslice, reslice);
    reslice->SetInput(volume_copy);
    reslice->SetOutputDimensionality(2);
    reslice->SetResliceAxes(reslice_axes);
    reslice->SetInterpolationModeToLinear();

    VTK_NEW(vtkMarchingSquares, squares);
    squares->SetInputConnection(reslice->GetOutputPort());
    squares->SetNumberOfContours(1);
    squares->SetValue(0, 128.0);

    // Display the image
    //    VTK_NEW(vtkImageMapper, image_mapper);
    //    //image_mapper->SetInputConnection(reslice->GetOutputPort());
    //    image_mapper->SetInput(squares->GetOutput());
    //    image_mapper->SetColorWindow(255.0);
    //    image_mapper->SetColorLevel(127.5);

    //    VTK_NEW(vtkActor2D, actor);
    //    actor->SetMapper(image_mapper);

    float width = spacing[0] * (extent[1] - extent[0]);
    float height = spacing[1] * (extent[3] - extent[2]);

    VTK_NEW(vtkPolyData, frame);
    VTK_NEW(vtkPoints, corners);
    corners->SetNumberOfPoints(4);
    corners->SetPoint(0, origin[0] - width, origin[1] - height, 0);
    corners->SetPoint(1, origin[0] - width, origin[1] + height, 0);
    corners->SetPoint(2, origin[0] + width, origin[1] - height, 0);
    corners->SetPoint(3, origin[0] + width, origin[1] + height, 0);
    frame->SetPoints(corners);

    VTK_NEW(vtkVertexGlyphFilter, glyph);
    glyph->SetInput(frame);

    VTK_NEW(vtkPolyDataMapper, frame_mapper);
    frame_mapper->SetInputConnection(glyph->GetOutputPort());
    frame_mapper->SetScalarVisibility(0);

    VTK_NEW(vtkActor, frame_actor);
    frame_actor->SetMapper(frame_mapper);
    frame_actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

    VTK_NEW(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(squares->GetOutputPort());
    mapper->SetScalarVisibility(0);

    VTK_NEW(vtkActor, actor);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);

    VTK_NEW(vtkRenderer, renderer);
    renderer->AddActor(actor);
    renderer->AddActor(frame_actor);
    renderer->ResetCamera();

    // Set up the interaction
    VTK_NEW(vtkInteractorStyleImage, image_style);
    VTK_NEW(vtkRenderWindowInteractor, interactor);
    interactor->SetInteractorStyle(image_style);

    VTK_NEW(vtkRenderWindow, window);
    window->AddRenderer(renderer);
    window->SetInteractor(interactor);
    window->Render();

    VTK_NEW(vtkImageInteractionCallback, callback);
    callback->SetImageReslice(reslice);
    callback->SetInteractor(interactor);
    callback->SetMarchingSquares(squares);
    callback->SetRenderer(renderer);

    image_style->AddObserver(vtkCommand::MouseMoveEvent, callback);
    image_style->AddObserver(vtkCommand::MouseWheelForwardEvent, callback);
    image_style->AddObserver(vtkCommand::MouseWheelBackwardEvent, callback);
    image_style->AddObserver(vtkCommand::LeftButtonPressEvent, callback);
    image_style->AddObserver(vtkCommand::LeftButtonReleaseEvent, callback);

    // Start interaction
    // The Start() method doesn't return until the window is closed by the user
    interactor->Start();

    return callback->GetThreshold();
  }

 private:
  SliceViewer();
};

#endif  // SLICEINTERACTOR_H
