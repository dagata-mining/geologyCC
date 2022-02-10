//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qM3C2                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#            COPYRIGHT: UNIVERSITE EUROPEENNE DE BRETAGNE                #
//#                                                                        #
//##########################################################################

#ifndef LIBPOINTMATCHER_PROCESS_HEADER
#define LIBPOINTMATCHER_PROCESS_HEADER

//Local
#include "LibpointmatcherDialog.h"
#include "LibpointmatcherOutlierDialog.h"
#include "LibpointmatcherConvergenceDialog.h"
//qCC
#include "ccMainAppInterface.h"
#include "ccStdPluginInterface.h"

class ccMainAppInterface;

//!

class LibpointmatcherProcess
{
public:
	
	static bool Subsample(const LibpointmatcherDialog& dlg, 
						ccHObject* entity,
						QString& errorMessage,
						QWidget* parentWidget/*=nullptr*/,
						ccMainAppInterface* app/*=nullptr*/);
	static ccGLMatrixd ICP(const LibpointmatcherOutlierDialog& dlg,
		QString& errorMessage,
		QWidget* parentWidget/*=nullptr*/,
		ccMainAppInterface* app/*=nullptr*/);
	static std::vector<ccGLMatrixd> convergence(const LibpointmatcherConvergenceDialog& dlg,
		QString& errorMessage,
		QWidget* parentWidget/*=nullptr*/,
		ccMainAppInterface* app/*=nullptr*/);
	// Transfrom T eigen matrix to a CCGLMatrix
	static ccGLMatrixd convertingOutputMatrix(Eigen::MatrixXf m);
	static bool Compute(const LibpointmatcherConvergenceDialog& dlg,
		QString& errorMessage,
		ccPointCloud* cloud1,
		ccPointCloud* cloud2,
		bool allowDialogs,
		QWidget* parentWidget = nullptr,
		ccMainAppInterface* app = nullptr);

};


#endif //LIBPOINTMATCHER_PROCESS_HEADER
