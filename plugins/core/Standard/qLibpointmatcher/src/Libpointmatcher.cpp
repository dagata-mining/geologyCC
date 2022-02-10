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

#include "Libpointmatcher.h"

//Qt
#include <QMainWindow>
#include <QProgressDialog>

//local
#include "LibpointmatcherTools.h"
#include "LibpointmatcherDialog.h"
#include "LibpointmatcherOutlierDialog.h"
#include "LibpointmatcherConvergenceDialog.h"
#include "LibpointmatcherDisclaimerDialog.h"
#include "LibpointmatcherProcess.h"

//qCC_db
#include <ccPointCloud.h>


Libpointmatcher::Libpointmatcher(QObject* parent)
	: QObject(parent)
	, ccStdPluginInterface( ":/CC/plugin/qLibpointmatcher/info.json" )

	, m_actionFilter(nullptr)
	, m_actionICP(nullptr)
	,m_actionConvergence(nullptr)
{
}

void Libpointmatcher::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_actionFilter)
	{
		m_actionFilter->setEnabled(selectedEntities.size() >= 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
	}
	if (m_actionICP)
	{
		m_actionICP->setEnabled(selectedEntities.size() == 2 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD) && selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD));
	}
	if (m_actionConvergence)
	{
		m_actionConvergence->setEnabled(numberOfPointCloudEntity(selectedEntities) >= 2);
	}

	m_selectedEntities = selectedEntities;
}

QList<QAction *> Libpointmatcher::getActions()
{
	if (!m_actionFilter)
	{
		m_actionFilter = new QAction("Subsample",this);
		m_actionFilter->setToolTip("Subsample with Libpointmatcher chains");
		m_actionFilter->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qLibpointmatcher/images/filterIcon.png")));
		connect(m_actionFilter, &QAction::triggered, this, &Libpointmatcher::doActionFilter);
	}

	if (!m_actionICP)
	{
		m_actionICP = new QAction("ICP", this);
		m_actionICP->setToolTip("ICP with Libpointmatcher chains");
		m_actionICP->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qLibpointmatcher/images/ICPIcon.png")));
		connect(m_actionICP, &QAction::triggered, this, &Libpointmatcher::doActionICP);
	}
	if (!m_actionConvergence)
	{
		m_actionConvergence = new QAction("Convergence", this);
		m_actionConvergence->setToolTip("Convergence with Libpointmatcher/M3C2 chains");
		m_actionConvergence->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qLibpointmatcher/images/convergenceIcon.png")));
		connect(m_actionConvergence, &QAction::triggered, this, &Libpointmatcher::doActionConvergence);
	}

	return QList<QAction *>{ m_actionFilter,
							m_actionICP,
							m_actionConvergence};
}

void Libpointmatcher::applyTransformationEntity(ccGLMatrixd transMat, ccHObject* entity)
{
	//if the transformation is partly converted to global shift/scale
	bool updateGlobalShiftAndScale = false;
	double scaleChange = 1.0;
	CCVector3d shiftChange(0, 0, 0);

	ccHObject::Container selectedEntities = m_selectedEntities;

	bool firstCloud = true;
	 //warning, getSelectedEntites may change during this loop!

		//we don't test primitives (it's always ok while the 'vertices lock' test would fail)
	if (!entity->isKindOf(CC_TYPES::PRIMITIVE))
	{
		//specific test for locked vertices
		bool lockedVertices;
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
		if (cloud)
		{


			if (firstCloud)
			{
				//test if the translated cloud was already "too big"
				//(in which case we won't bother the user about the fact
				//that the transformed cloud will be too big...)
				ccBBox localBBox = entity->getOwnBB();
				CCVector3d Pl = CCVector3d::fromArray(localBBox.minCorner().u);
				double Dl = localBBox.getDiagNormd();

				//the cloud was alright
				if (!ccGlobalShiftManager::NeedShift(Pl)
					&& !ccGlobalShiftManager::NeedRescale(Dl))
				{
					//test if the translated cloud is not "too big" (in local coordinate space)
					ccBBox rotatedBox = entity->getOwnBB() * transMat;
					double Dl2 = rotatedBox.getDiagNorm();
					CCVector3d Pl2 = CCVector3d::fromArray(rotatedBox.getCenter().u);

					bool needShift = ccGlobalShiftManager::NeedShift(Pl2);
					bool needRescale = ccGlobalShiftManager::NeedRescale(Dl2);

					if (needShift || needRescale)
					{
						//existing shift information
						CCVector3d globalShift = cloud->getGlobalShift();
						double globalScale = cloud->getGlobalScale();

						//we compute the transformation matrix in the global coordinate space
						ccGLMatrixd globalTransMat = transMat;
						globalTransMat.scaleRotation(1.0 / globalScale);
						globalTransMat.setTranslation(globalTransMat.getTranslationAsVec3D() - globalShift);
						//and we apply it to the cloud bounding-box
						ccBBox rotatedBox = cloud->getOwnBB() * globalTransMat;
						double Dg = rotatedBox.getDiagNorm();
						CCVector3d Pg = CCVector3d::fromArray(rotatedBox.getCenter().u);

						//ask the user the right values!
						ccShiftAndScaleCloudDlg sasDlg(Pl2, Dl2, Pg, Dg);
						sasDlg.showApplyAllButton(false);
						sasDlg.showTitle(true);
						sasDlg.setKeepGlobalPos(true);
						sasDlg.showKeepGlobalPosCheckbox(false); //we don't want the user to mess with this!
						sasDlg.showPreserveShiftOnSave(true);

						//add "original" entry
						int index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo("Original", globalShift, globalScale));
						//sasDlg.setCurrentProfile(index);
						//add "suggested" entry
						CCVector3d suggestedShift = ccGlobalShiftManager::BestShift(Pg);
						double suggestedScale = ccGlobalShiftManager::BestScale(Dg);
						index = sasDlg.addShiftInfo(ccGlobalShiftManager::ShiftInfo("Suggested", suggestedShift, suggestedScale));
						sasDlg.setCurrentProfile(index);
						//add "last" entry (if available)
						std::vector<ccGlobalShiftManager::ShiftInfo> lastInfos;
						if (ccGlobalShiftManager::GetLast(lastInfos))
						{
							sasDlg.addShiftInfo(lastInfos);
						}
						//add entries from file (if any)
						sasDlg.addFileInfo();

						if (sasDlg.exec())
						{
							//store the shift for next time!
							double newScale = sasDlg.getScale();
							CCVector3d newShift = sasDlg.getShift();
							ccGlobalShiftManager::StoreShift(newShift, newScale);

							//get the relative modification to existing global shift/scale info
							assert(cloud->getGlobalScale() != 0);
							scaleChange = newScale / cloud->getGlobalScale();
							shiftChange = newShift - cloud->getGlobalShift();

							updateGlobalShiftAndScale = (scaleChange != 1.0 || shiftChange.norm2() != 0);

							//update transformation matrix accordingly
							if (updateGlobalShiftAndScale)
							{
								transMat.scaleRotation(scaleChange);
								transMat.setTranslation(transMat.getTranslationAsVec3D() + newScale * shiftChange);
							}
						}
						else if (sasDlg.cancelled())
						{
							ccLog::Warning("[ApplyTransformation] Process cancelled by user");
							return;
						}
					}
				}

				firstCloud = false;
			}

			if (updateGlobalShiftAndScale)
			{
				//apply translation as global shift
				cloud->setGlobalShift(cloud->getGlobalShift() + shiftChange);
				cloud->setGlobalScale(cloud->getGlobalScale() * scaleChange);
				const CCVector3d& T = cloud->getGlobalShift();
				double scale = cloud->getGlobalScale();
				ccLog::Warning(QString("[ApplyTransformation] Cloud '%1' global shift/scale information has been updated: shift = (%2,%3,%4) / scale = %5").arg(cloud->getName()).arg(T.x).arg(T.y).arg(T.z).arg(scale));
			}
		}
	}

	//we temporarily detach entity, as it may undergo
	//"severe" modifications (octree deletion, etc.) --> see ccHObject::applyRigidTransformation
	ccMainAppInterface::ccHObjectContext objContext = m_app->removeObjectTemporarilyFromDBTree(entity);
	entity->setGLTransformation(ccGLMatrix(transMat.data()));
	//DGM FIXME: we only test the entity own bounding box (and we update its shift & scale info) but we apply the transformation to all its children?!
	entity->applyGLTransformation_recursive();
	entity->prepareDisplayForRefresh_recursive();
	m_app->putObjectBackIntoDBTree(entity, objContext);

	 
	m_selectedEntities = selectedEntities;
	ccLog::Print("[ApplyTransformation] Applied transformation matrix:");
	ccLog::Print(transMat.toString(12, ' ')); //full precision
	ccLog::Print("Hint: copy it (CTRL+C) and apply it - or its inverse - on any entity with the 'Edit > Apply transformation' tool");
	
	m_app->refreshAll();
}

void Libpointmatcher::doActionFilter()
{
	//disclaimer accepted?
	if (!DisclaimerDialog::show(m_app))
		return;
	
	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (m_selectedEntities.size() < 1 || !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a pointcloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//display dialog
	LibpointmatcherDialog dlg(m_app);
	if (!dlg.exec())
	{
		//process cancelled by the user
		return;
	}
	// verify on which widget you are
	if (dlg.getCurrentFilterTabWidget() == 0 && dlg.getFilters().size()==0)
	{
		
		dlg.acceptNormalOptions();
		dlg.acceptFilterOptions();
		
	}
	if (dlg.getCurrentFilterTabWidget() == 1 && dlg.getFilters().size() == 0)
	{
		return;
	}
	if (dlg.getCurrentFilterTabWidget() == 2 && dlg.getFilters().size() == 0)
	{
		return;
	}

	QString errorMessage;
	ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
	QProgressDialog pDlg("Please wait...", "Cancel", 0, 0);
	pDlg.setWindowTitle("Libpointmatcher");
	pDlg.show();
	for (int i = 0; i < m_selectedEntities.size(); i++) {
		if (!pDlg.wasCanceled()) {
			if (!LibpointmatcherProcess::Subsample(dlg, m_selectedEntities[i], errorMessage, m_app->getMainWindow(), m_app))
			{
				m_app->dispToConsole(errorMessage, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			}
		}
		else 
		{
			break;
		}
	}
	if (m_app)
	{
		m_app->refreshAll();
		pDlg.reset();
	}

}

void Libpointmatcher::doActionICP()
{
	//disclaimer accepted?
	if (!DisclaimerDialog::show(m_app))
		return;

	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (m_selectedEntities.size() != 2 || !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD) || !m_selectedEntities[1]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select two pointclouds", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//display dialog
	ccPointCloud* cloud1 = ccHObjectCaster::ToPointCloud(m_selectedEntities[0]);
	ccPointCloud* cloud2 = ccHObjectCaster::ToPointCloud(m_selectedEntities[1]);
	LibpointmatcherOutlierDialog dlgICP(cloud1,cloud2,m_app);
	if (!dlgICP.exec())
	{
		//process cancelled by the user
		return;
	}
	if (!dlgICP.nofilterAllowed())
	{
		if(dlgICP.getFiltersRef().size() < 1 || dlgICP.getFiltersRead().size() < 1)
		{
			m_app->dispToConsole(QString("No Subsampling filters selected, enable no filter option or add filters "), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}
	// verify on which widget you are
	 
	dlgICP.acceptNormalOptions();
	dlgICP.acceptOutlierOption();
	dlgICP.acceptKdTreeOption();
	dlgICP.acceptMinimizerOption();
	dlgICP.acceptCheckerOption();
	ccLog::Print(QString("After Checker"));
	

	QString errorMessage;
	ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
	QProgressDialog pDlg("Please wait...", "Cancel", 0, 0);
	pDlg.setWindowTitle("Libpointmatcher");
	pDlg.show();

	if (!pDlg.wasCanceled())
	{
		ccGLMatrixd mat = LibpointmatcherProcess::ICP(dlgICP, errorMessage, m_app->getMainWindow(), m_app);
		applyTransformationEntity(mat, m_selectedEntities[dlgICP.getCurrentreadIndexEntity()]);
		
	}
	if (m_app)
	{
		m_app->refreshAll();
		pDlg.reset();
	}

}
;
void Libpointmatcher::doActionConvergence()
{
	//disclaimer accepted?
	if (!DisclaimerDialog::show(m_app))
		return;

	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (m_selectedEntities.size() < 2 || !m_selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select two pointclouds or more", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	LibpointmatcherConvergenceDialog dlgConvergence(m_selectedEntities, m_app);
	if (!dlgConvergence.exec())
	{
		//process cancelled by the user
		return;
	}
	if (!dlgConvergence.nofilterAllowed())
	{
		if (dlgConvergence.getFiltersRef().size() < 1 || dlgConvergence.getFiltersRead().size() < 1)
		{
			m_app->dispToConsole(QString("No Subsampling filters selected, enable no filter option or add filters "), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}
	}
	// verify on which widget you are

	dlgConvergence.acceptNormalOptions();
	dlgConvergence.acceptOutlierOption();
	dlgConvergence.acceptKdTreeOption();
	dlgConvergence.acceptMinimizerOption();
	dlgConvergence.acceptCheckerOption();
	

	QString errorMessage;
	ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
	QProgressDialog pDlg("Please wait...", "Cancel", 0, 0);
	pDlg.setWindowTitle("Libpointmatcher");
	pDlg.show();

	

	if (!pDlg.wasCanceled())
	{
		
		std::vector<ccGLMatrixd> transformationList =
			LibpointmatcherProcess::convergence(dlgConvergence, errorMessage, m_app->getMainWindow(), m_app);
		if (transformationList.size() > 0)
		{
			for (int i = 0; i < transformationList.size(); i++)
			{
				applyTransformationEntity(transformationList[i], m_selectedEntities[dlgConvergence.getSliceListIndexes()[i]]);
				
					
			}
			if (dlgConvergence.calculateM3C2->isChecked()) 
			{
				for (int i = 0; i < transformationList.size(); i++)
				{
					bool a = LibpointmatcherProcess::Compute(dlgConvergence, errorMessage, dlgConvergence.getSliceList()[i], dlgConvergence.getCloudRefConvergence(), true, m_app->getMainWindow(), m_app);
				}
				for (int i = 0; i < transformationList.size(); i++)
				{
					applyTransformationEntity(transformationList[i].inverse(), m_selectedEntities[dlgConvergence.getSliceListIndexes()[i]]);
				}
				

			}

		}
		else return;
	}
	if (m_app)
	{
		m_app->refreshAll();
		pDlg.reset();
	}

}
int Libpointmatcher::numberOfPointCloudEntity(std::vector<ccHObject*> entities)
{
	int n=0;
	for (int i = 0; i < entities.size(); i++)
	{
		if (entities[i]->isA(CC_TYPES::POINT_CLOUD))
		{
			n++;
		};
	}
	return n;
}

;


