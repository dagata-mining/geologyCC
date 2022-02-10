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

#ifndef LIBPOINTMATCHER_PLUGIN_HEADER
#define LIBPOINTMATCHER_PLUGIN_HEADER

//qCC
#include "ccStdPluginInterface.h"
#include "ccMainAppInterface.h"

//qCC_db
#include <ccHObject.h>

#include <ccGlobalShiftManager.h>
#include <ccShiftAndScaleCloudDlg.h>



//! Libpointmatcher
/** Libpointmatcher 


**/
class Libpointmatcher : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )

	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qLibpointmatcher" FILE "../info.json" )

public:

	//! Default constructor
	Libpointmatcher(QObject* parent = nullptr);

	virtual ~Libpointmatcher() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;
	

private:
	//! Subsample launching
	void doActionFilter();
	//! ICP launching
	void doActionICP();
	//! Convergence launching
	void doActionConvergence();
	//! Applying Transformation 
	void applyTransformationEntity(ccGLMatrixd, ccHObject*);
	//! Number of Point Clouds
	int numberOfPointCloudEntity(std::vector<ccHObject*>);
	


	//! Default action
	QAction* m_actionFilter;
	QAction* m_actionICP;
	QAction* m_actionConvergence;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //LIBPOINTMATCHER_PLUGIN_HEADER
