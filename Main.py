import sys
sys.path.append("D:\\OneDrive\\ToyLabsP4v\\PythonWorks\\HelperBones")
# sys.path.append("C:\\Users\\xxiao2\\OneDrive\\ToyLabsP4v\\PythonWorks\\\HelperBones")


from pymxs import runtime as rt
import Utilities as utils
reload(utils)

import Config as config
reload(config)

import Learner as ln
reload(ln)


def createHelperBoneSystem():
    sysOption = config.SystemOption()

    regularMesh = rt.getNodeByName(sysOption.regularMeshName)
    referenceMesh = rt.getNodeByName(sysOption.referenceMeshName)

    primarySkinMod = 0

    for i in range(0, len(regularMesh.Modifiers)):
        if str(rt.classof(regularMesh.Modifiers[i])) == 'Skin':
            regularMesh.Modifiers[i].name = 'Skin_Original'
            primarySkinMod = regularMesh.Modifiers[i]
            break

    poseLearner = ln.PoseLearner(regularMesh, referenceMesh, primarySkinMod, sysOption)
    poseLearner.processStart()

    # re-select the mesh to refresh the modifiers list displaying
    rt.select(regularMesh)



import re

def learnMotion():
    sysOption = config.SystemOption()

    learningBoneList = []
    referenceBoneList = []

    for nd in rt.helpers:
        pattern = re.compile(sysOption.helperBonePrefix + ".*")
        if pattern.match(nd.Name):
            learningBoneList.append(nd)


    for bnName in sysOption.tempReferenceBoneList:
        bn = rt.getNodeByName(bnName)
        referenceBoneList.append(bn)

    motionLearner = ln.MotionLearner(learningBoneList, referenceBoneList, sysOption)

    motionLearner.processStart()



# createHelperBoneSystem()
learnMotion()