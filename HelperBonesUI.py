import sys
from PySide import QtGui, QtCore
import MaxPlus as mp
from pymxs import runtime as rt

import copy

sys.path.append("D:\\OneDrive\\ToyLabsP4v\\PythonWorks\\HelperBones")
# sys.path.append("C:\\Users\\xxiao2\\OneDrive\\ToyLabsP4v\\PythonWorks\\\HelperBones")

import Utilities as utils
reload(utils)

import Config as config
reload(config)

import Learner as ln
reload(ln)

class LODHelpersUI (QtGui.QWidget):
    def __init__(self, parent):
        super(LODHelpersUI, self).__init__(parent)
        self.util = utils.Utility()
        self.sysOption = config.SystemOption()

        ###### Create the UI ######
        layoutMain = QtGui.QVBoxLayout()


        self.btnCreateHelperBones = QtGui.QPushButton("Create Helper Bones")
        layoutMain.addWidget(self.btnCreateHelperBones)

        # Translation Params #
        layoutTranslateParams= QtGui.QVBoxLayout()
        layoutTranslateParams.addWidget(QtGui.QLabel('Translation Params'))

        # tolerance params
        layoutTranslateTolError = QtGui.QHBoxLayout()
        layoutTranslateTolError.addWidget(QtGui.QLabel('Tolerant Error'))
        self.spnTol = QtGui.QSpinBox()
        layoutTranslateTolError.addWidget(self.spnTol)
        layoutTranslateParams.addLayout(layoutTranslateTolError)

        # steps params
        layoutTranaslateStep = QtGui.QHBoxLayout()
        layoutTranaslateStep.addWidget(QtGui.QLabel('Step Range'))

        self.spnStepRangeMin = QtGui.QSpinBox()
        layoutTranaslateStep.addWidget(self.spnStepRangeMin)

        self.spnStepRangeMax = QtGui.QSpinBox()
        layoutTranaslateStep.addWidget(self.spnStepRangeMax)

        layoutTranslateParams.addLayout(layoutTranaslateStep)
        layoutMain.addLayout(layoutTranslateParams)

        self.setLayout(layoutMain)

        ###### Link the UI to Functions ######
        self.btnCreateHelperBones.clicked.connect(self.createHelperBoneSystem)

    def createHelperBoneSystem(self):
        regularMesh = rt.getNodeByName('Regular')
        referenceMesh = rt.getNodeByName('Reference')

        primarySkinMod = 0

        for i in range(0, len(regularMesh.Modifiers)):
            if str(rt.classof(regularMesh.Modifiers[i])) == 'Skin':
                regularMesh.Modifiers[i].name = "Skin_Backup"
                primarySkinMod = regularMesh.Modifiers[i]
                break

        # primaryBoneNameList = ['Bip001 L Clavicle', 'Bip001 L UpperArm', 'Bip001 L Forearm']
        primaryBoneNameList = ['Root']
        poseLearner = ln.PoseLearner(regularMesh, referenceMesh, primarySkinMod, primaryBoneNameList, self.sysOption)

        poseLearner.processStart()

        # re-select the mesh to refresh the modifiers list displaying
        rt.select(regularMesh)


class MainWindow(QtGui.QMainWindow):
    def __init__(self, parent):
        super(MainWindow, self).__init__(parent)
        # self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

        ui = LODHelpersUI(self)

        self.setCentralWidget(ui)


maxWindow = mp.GetQMaxWindow()
mainWindow = MainWindow(maxWindow)
mainWindow.show()
mainWindow.move(mainWindow.pos().x() + 500, mainWindow.pos().y())


########## UI #############
class _GCProtector(object):
    widgets = []

_GCProtector.widgets.append(mainWindow)