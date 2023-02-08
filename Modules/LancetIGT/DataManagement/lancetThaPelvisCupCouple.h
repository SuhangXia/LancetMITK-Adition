
#ifndef THAPELVISCUPCOUPLE_H
#define THAPELVISCUPCOUPLE_H

#include <mitkImageToImageFilter.h>

// The following header file is generated by CMake and thus it's located in
// the build directory. It provides an export macro for classes and functions
// that you want to be part of the public interface of your module.
#include <MitkLancetIGTExports.h>

#include "lancetThaCupObject.h"
#include "lancetThaPelvisObject.h"
#include "mitkDataNode.h"
#include "mitkPointSet.h"
#include "mitkSurface.h"

namespace lancet
{
	/**Documentation
	  * \brief 
	  *
	  * \ingroup IGT
	  */
	class MITKLANCETIGT_EXPORT ThaPelvisCupCouple : public itk::DataObject
	{
	public:
		mitkClassMacroItkParent(ThaPelvisCupCouple, itk::DataObject);
		itkFactorylessNewMacro(Self)
		itkCloneMacro(Self)

		itkGetMacro(PelvisObject, lancet::ThaPelvisObject::Pointer);
		itkSetMacro(PelvisObject, lancet::ThaPelvisObject::Pointer);

		itkGetMacro(CupObject, lancet::ThaCupObject::Pointer);
		itkSetMacro(CupObject, lancet::ThaCupObject::Pointer);

		/*
		 * m_vtkMatrix_coupleGeometry remains the same
		 * m_vtkMatrix_groupGeometry of m_PelvisObject remains the same
		 * update m_vtkMatrix_groupGeometry of m_CupObject
		 * update m_pelvisFrameToCupFrame
		 * call UpdateCupAngles()
		 * call UpdateRelativeCupCOR()
		 */
		void SetPelvisFrameToCupFrameMatrix(vtkSmartPointer<vtkMatrix4x4> newMatrix);
		itkGetMacro(vtkMatrix_pelvisFrameToCupFrame, vtkSmartPointer<vtkMatrix4x4>);
		
		itkGetMacro(CupInclination_supine, double);
		
		itkGetMacro(CupInclination_stand, double);
	
		itkGetMacro(CupInclination_sit, double);

		itkGetMacro(CupInclination_noTilt, double);

		itkGetMacro(CupVersion_noTilt, double);

		itkGetMacro(CupVersion_supine, double);
		
		itkGetMacro(CupVersion_stand, double);
		
		itkGetMacro(CupVersion_sit, double);
	

		itkGetMacro(CupCOR_SI_supine, double);
		itkGetMacro(CupCOR_ML_supine, double);
		itkGetMacro(CupCOR_AP_supine, double);

		itkGetMacro(CupCOR_SI_noTilt, double);
		itkGetMacro(CupCOR_ML_noTilt, double);
		itkGetMacro(CupCOR_AP_noTilt, double);

		/* update the m_vtkMatrix_groupGeometry of m_PelvisObject and m_CupObject;
		 * update m_vtkMatrix_coupleGeometry */
		void SetCoupleGeometry(vtkSmartPointer<vtkMatrix4x4> newMatrix);
		itkGetMacro(vtkMatrix_coupleGeometry, vtkSmartPointer<vtkMatrix4x4>);

		// initialize pelvisFrame to cupFrame matrix and move the cupObject to the initial position 
		void InitializePelvisFrameToCupFrameMatrix();

		/* Translate or rotate cup
		 * translateOrRotate: translate(0), rotate(1)
		 * direction: x-axis(0), y-axis(1), z-axis(2)
		 * step: mm or degree
		 */
		void AdjustCup(int translateOrRotate, int direction, double step);

	protected:

		ThaPelvisCupCouple();
		ThaPelvisCupCouple(const ThaPelvisCupCouple& other);
		~ThaPelvisCupCouple() override;
		
		// Update cup inclination and cup version, called by AdjustCup()
		void UpdateCupAngles();
		// Update m_CupCOR_SI, m_CupCOR_ML and m_CupCOR_AP, called by AdjustCup()
		void UpdateRelativeCupCOR();

		// Translate cup: x axis of pelvisFrame
		void TranslateCup_x(double length);
		// Translate cup: y axis of pelvisFrame
		void TranslateCup_y(double length);
		// Translate cup: z axis of pelvisFrame
		void TranslateCup_z(double length);
		// Rotate cup in degree (the rotation axis passes through the cupFrame origin): x axis of pelvisFrame
		void RotateCup_x(double angle);
		// Rotate cup in degree (the rotation axis passes through the cupFrame origin): y axis of pelvisFrame
		void RotateCup_y(double angle);
		// Rotate cup in degree (the rotation axis passes through the cupFrame origin): z axis of pelvisFrame
		void RotateCup_z(double angle);

		// the pelvisObject
		lancet::ThaPelvisObject::Pointer m_PelvisObject;

		// the cupObject
		lancet::ThaCupObject::Pointer m_CupObject;

		// pelvisObject internal frame to cupObject internal frame
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_pelvisFrameToCupFrame;

		// the geometry matrix of the pelvis-cup couple
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_coupleGeometry;

		// cup inclination: supine
		double m_CupInclination_supine{ 0 };
		// cup inclination: stand
		double m_CupInclination_stand{ 0 };
		// cup inclination: sit
		double m_CupInclination_sit{ 0 };
		// cup inclination: no pelvic tilt
		double m_CupInclination_noTilt{ 0 };
		// cup version: supine
		double m_CupVersion_supine{ 0 };
		// cup version: stand
		double m_CupVersion_stand{ 0 };
		// cup version: sit
		double m_CupVersion_sit{ 0 };
		// cup version: no pelvic tilt
		double m_CupVersion_noTilt{ 0 };

		// cupCOR relative to pelvisCOR with supine pelvic tilt: Superior(+) or inferior(-)
		double m_CupCOR_SI_supine{ 0 };
		// cupCOR relative to pelvisCOR with supine pelvic tilt: Medial(+) or lateral(-) 
		double m_CupCOR_ML_supine{ 0 };
		// cupCOR relative to pelvisCOR with supine pelvic tilt: Anterior(+) or posterior(-)
		double m_CupCOR_AP_supine{ 0 };

		// cupCOR relative to pelvisCOR with no pelvic tilt: Superior(+) or inferior(-)
		double m_CupCOR_SI_noTilt{ 0 };
		// cupCOR relative to pelvisCOR with no pelvic tilt: Medial(+) or lateral(-) 
		double m_CupCOR_ML_noTilt{ 0 };
		// cupCOR relative to pelvisCOR with no pelvic tilt: Anterior(+) or posterior(-)
		double m_CupCOR_AP_noTilt{ 0 };

	};
}

#endif
