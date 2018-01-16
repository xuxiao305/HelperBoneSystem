from PySide import QtGui, QtCore
import MaxPlus as mp
from pymxs import runtime as rt

import copy

import Utilities as utils
reload(utils)

import Config as config
reload(config)

import MeshData as dt
reload(dt)


import Performance as perf
reload(perf)

import MyMath as math1
reload(math1)


debugLog = False

import sys
sys.path.append("D:\\OneDrive\\ToyLabsP4v\\PythonWorks\\HelperBones")

import time

class PoseLearner:
    def __init__(self, regularMesh, referenceMesh, primarySkinMod, option):
        self.option = option
        self.perfEval = perf.PerformanceEvaluator()
        self.util = utils.Utility()
        self.skinModUtil = utils.SkinModUtility()
        self.computeUtil = utils.ComputingUtility()
        self.vertUtil = utils.VertexUtility()

        self.regularMeshData = dt.RegularMeshData(regularMesh, primarySkinMod, option.primaryBoneNameList, self.option)
        self.referenceMeshData = dt.ReferenceMeshData(referenceMesh, self.option)

        # Learner's learning parameters
        self.skinningParms = option.skinningParms
        self.translateParms = option.translateParms
        self.rotateParms = option.rotateParms

        self.totalError = 0

    def processStart(self):
        print '--- PROCESS START ---'
        mp.Core.EvalMAXScript('set animate on')


        while len(self.regularMeshData.helperBoneList) < self.option.maxHelperBoneCount:
            print 'self.regularMeshData.helperBoneList', self.regularMeshData.helperBoneList
            newHelperBone, neighborVertsCore, neighborVertsOuter = self.addNewHelperBoneToWorstVertexList()

            avgError = self.computeUtil.computeAvgErrorForAllVerts(
                self.regularMeshData.allVertPosList, self.referenceMeshData.allVertPosList, self.option.frames)
            self.perfEval.newError(avgError, 'A NEW BONE ADDED')

            self.regularMeshData.includeNewBone(newHelperBone, neighborVertsCore, neighborVertsOuter)

            self.IterateThrough(self.option.iteration)

        self.IterateThrough(100)
        self.perfEval.logResult()

        self.regularMeshData.setLearntBoneXF()
        self.regularMeshData.setLearntSkinMod()

    def addNewHelperBoneToWorstVertexList(self):
        worstVertIndex, worstVertPos, worstError = \
            self.computeUtil.findVertWithMaxError(self.regularMeshData.allVertPosList, self.referenceMeshData.allVertPosList)

        neighborVertsCore = self.vertUtil.getNeighborVertex(self.regularMeshData.obj, worstVertIndex)

        neighborVertsOuter = []
        for v in neighborVertsCore:
            neighbors = self.vertUtil.getNeighborVertex(self.regularMeshData.obj, v)
            for n in neighbors:
                if self.util.findInList(neighborVertsCore, n) == -1:
                    neighborVertsOuter.append(n)

        neighborVertsOuter2 = []
        for v in neighborVertsOuter:
            neighbors = self.vertUtil.getNeighborVertex(self.regularMeshData.obj, v)
            for n in neighbors:
                if self.util.findInList(neighborVertsCore + neighborVertsOuter, n) == -1:
                    neighborVertsOuter2.append(n)

        neighborVertsOuterAll = self.util.makeUnique(neighborVertsOuter + neighborVertsOuter2)

        avgPos = self.vertUtil.getAvgPosFromVertex(self.regularMeshData.obj, neighborVertsCore)

        newBn = rt.Point(position = avgPos, wireColor = rt.Point3(0,255,0), size = 8, box = True, cross=False, name='Point_')
        newBn.Name = self.option.helperBonePrefix + newBn.Name + str(len(self.regularMeshData.helperBoneList) )

        # newBn.parent = rt.getNodeByName('Bip001 L UpperArm')

        return newBn, neighborVertsCore, neighborVertsOuterAll

    def IterateThrough(self, iteration):
        for i in range(0, iteration):
            self.perfEval.newIteration()

            ######## Optimize the skinning and set the result back to regularMeshData's allWeightList ########
            skinImproved, newAllVertWeight, newAllVertError = self.optimizeSkinning()
            self.regularMeshData.allWeightList = newAllVertWeight

            avgError = self.computeUtil.computeAvgErrorForAllVerts(
                self.regularMeshData.allVertPosList, self.referenceMeshData.allVertPosList, self.option.frames)
            self.perfEval.newError(avgError, 'skin')

            ######## Optimize the transform and set the result back to regularMeshData's boneXFList ########
            # For Translation
            translationImproved = self.optimizeTransform()
            avgError = self.computeUtil.computeAvgErrorForAllVerts(
                self.regularMeshData.allVertPosList, self.referenceMeshData.allVertPosList, self.option.frames)
            self.perfEval.newError(avgError, 'translate')

            # For Rotation
            rotationImproved = self.optimizeTransform(transformMode ='rotate')
            avgError = self.computeUtil.computeAvgErrorForAllVerts(
                self.regularMeshData.allVertPosList, self.referenceMeshData.allVertPosList, self.option.frames)
            self.perfEval.newError(avgError, 'rotation')

            if not skinImproved and not translationImproved and not rotationImproved:
                print 'NO IMPROVEMENT MADE, READY FOR NEXT BONE OR FINISH LEARNING'
                print 'i, skin, translation, rotation', i, skinImproved, translationImproved, rotationImproved
                return


    def optimizeSkinning(self):
        improved = False
        # print '\n\n-- OPTIMIZE SKINNING --'
        # ok let's process all vertex one by one...
        outAllWeightList = []
        outAllError = []

        for v in range(0, len(self.regularMeshData.obj.verts)):
            # print 'v--->, ', v

            vert = self.regularMeshData.obj.verts[v]

            vertBoneList = self.regularMeshData.allVertBoneList[v]
            transformList = self.regularMeshData.queryMutilVertBoneTransformList(vertBoneList)

            referenceVertPosList = self.referenceMeshData.allVertPosList[v]

            # this is our new weight list
            newVertWeightList = self.regularMeshData.allWeightList[v]


            # get vertex positions in all frames before simulation

            preSimVertPosList = self.computeUtil.TransformPosition(vert.pos, newVertWeightList, transformList, self.option.frames)

            preSimError = self.computeUtil.computeErrorForOneVert(preSimVertPosList, referenceVertPosList)
            # print " preSimError-->", preSimError

            if (preSimError / self.option.frames) < self.option.tolerantError:
                # print "the error is small enough, go to next vertex", preSimError
                outAllWeightList.append(newVertWeightList)
                outAllError.append(preSimError)
                continue

            # deep copy the weight into a testing weight list
            simWeightList = copy.deepcopy(newVertWeightList)

            # print 'simWeightList, ', simWeightList

            # try change the weight and compare the error,
            # if the change reduce the error, then use it as the newVertWeightList,
            # if not, then discard it and try the next weight value along the list
            avgError = preSimError / self.option.frames

            self.skinningParms.step = self.skinningParms.getAdaptiveStep(avgError)

            postSimError = 0
            for w in range(0, len(simWeightList)):
                # try to tweak the weight and put it in simulation
                simWeightList[w] += self.skinningParms.step

                simWeightList = self.util.normalizeListValue(simWeightList)

                simVertPosList = self.computeUtil.TransformPosition(vert.pos, simWeightList, transformList, self.option.frames)

                postSimError = self.computeUtil.computeErrorForOneVert(simVertPosList, referenceVertPosList)

                if preSimError - postSimError > self.option.tolerantError:
                    newVertWeightList = copy.deepcopy(simWeightList)
                    self.regularMeshData.allVertPosList[v] = simVertPosList
                    improved = True
                else:
                    simWeightList = copy.deepcopy(newVertWeightList)

            outAllWeightList.append(newVertWeightList)
            outAllError.append(postSimError)


        return improved, outAllWeightList, outAllError

    # EVERY bones in the bone list need to be updated, because the newly added bones might effect the other bone's best solution for transform
    def optimizeTransform(self, transformMode ='translate'):
        improved = False
        # print '\n-- OPTIMIZE TRANSLATION --'
        for b in range(0, len(self.regularMeshData.allBonesXFList)):
            bn = self.regularMeshData.allBoneList[b]
            if self.util.findInNodeList(self.regularMeshData.helperBoneList, bn) == -1:
                continue

            # calculate the error before simulation
            vertIndexSet = self.regularMeshData.getVertIndexByBone(bn)

            for f in range(1, self.option.frames):
                # This is quite brute force.. by trying the translation separately on positive and negative direction of the same axis
                for axis in range(0, 3):
                    # get sum error for vertex set
                    preSimError = 0
                    for v in vertIndexSet:
                        vertPos = self.regularMeshData.allVertPosList[v][f]
                        referenceVertPos = self.referenceMeshData.allVertPosList[v][f]

                        preSimError += self.computeUtil.computeErrorForOnePos(vertPos, referenceVertPos)

                    boneXF = self.regularMeshData.allBonesXFList[b][f]

                    avgError = preSimError / float(self.option.frames)
                    avgError = avgError / float(len(vertIndexSet))

                    if transformMode == 'translate':
                        self.translateParms.step = self.translateParms.getAdaptiveStep(avgError)
                        newBnMatrix, simVertSetPos, postError = \
                            self.stepOneAxis(self.translateParms.step, bn.Name, boneXF, axis, f, vertIndexSet, transformMode)
                    else:
                        self.rotateParms.step = self.rotateParms.getAdaptiveStep(avgError)
                        newBnMatrix, simVertSetPos, postError = \
                            self.stepOneAxis(self.rotateParms.step, bn.Name, boneXF, axis, f, vertIndexSet, transformMode)

                    # print 'post error is too big, try the other direction', postError

                    if postError > preSimError:
                        if transformMode == 'translate':
                            self.skinningParms.step = self.translateParms.getAdaptiveStep(avgError)
                            newBnMatrix, simVertSetPos, postError = \
                                self.stepOneAxis(-self.translateParms.step, bn.Name, boneXF, axis, f, vertIndexSet, transformMode)
                        else:
                            self.rotateParms.step = self.rotateParms.getAdaptiveStep(avgError)
                            newBnMatrix, simVertSetPos, postError = \
                                self.stepOneAxis(-self.rotateParms.step, bn.Name, boneXF, axis, f, vertIndexSet, transformMode)

                    if preSimError - postError > self.option.tolerantError:
                        self.regularMeshData.allBonesXFList[b][f] = newBnMatrix
                        for i in range(0, len(vertIndexSet)):
                            vertIndex = vertIndexSet[i]
                            self.regularMeshData.allVertPosList[vertIndex][f] = simVertSetPos[i]

                            improved = True

        return improved

    def stepOneAxis(self, step, bnName, boneXF, a, f, vertIndexSet, mode = 'translate'):
        boneNewXF = rt.Matrix3(boneXF[0], boneXF[1], boneXF[2], boneXF[3])

        if mode == 'translate':
            if a == 0:
                translationVector = rt.Point3(step, 0, 0)
            elif a == 1:
                translationVector = rt.Point3(0, step, 0)
            else:
                translationVector = rt.Point3(0, 0, step)
            boneNewXF = rt.translate(boneNewXF, translationVector)

        elif mode == 'rotate':
            if a == 0:
                rotMatrix = rt.rotateXMatrix (step)
            elif a == 1:
                rotMatrix = rt.rotateYMatrix(step)
            else:
                rotMatrix = rt.rotateZMatrix(step)

            # translate the matrix back to origin
            boneNewXFTranslation = boneNewXF.translation
            boneNewXF = rt.translate(boneNewXF, -boneNewXFTranslation)
            # make the rotation and translate it back
            boneNewXF = boneNewXF * rotMatrix
            boneNewXF = rt.translate(boneNewXF, boneNewXFTranslation)


        postSimError = 0

        simVertSetPos = []

        for v in vertIndexSet:
            preSimVertPos = self.regularMeshData.allVertPosList[v][f]

            weight = 0
            for vertBoneIndex in range(0, len(self.regularMeshData.allVertBoneList[v])):
                if self.regularMeshData.allVertBoneList[v][vertBoneIndex].Name == bnName:

                    weight = self.regularMeshData.allWeightList[v][vertBoneIndex]

            simVertPos = preSimVertPos * (1-weight) + preSimVertPos * rt.inverse(boneXF) * boneNewXF * weight

            simVertSetPos.append(simVertPos)

            referenceVertPos = self.referenceMeshData.allVertPosList[v][f]

            postSimError += self.computeUtil.computeErrorForOnePos(simVertPos, referenceVertPos)

        return boneNewXF, simVertSetPos, postSimError


# Motion Learner can use the Bone Pose Data into a polynomial, and set it into the bone's Rotation and Position controller
# the Bone Pose Data should be most likely coming from the Pose Learner, but we are trying to separate them here,
# # to make the modules reusable as possible.
class MotionLearner:
    def __init__(self, learningBonesList, referenceBonesList, option):
        self.option = option
        self.perfEval = perf.PerformanceEvaluator()
        self.util = utils.Utility()
        self.computeUtil = utils.ComputingUtility()
        self.math1 = math1.SimpleMath()
        self.vertUtil = utils.VertexUtility()

        self.helperBonesList = learningBonesList
        self.helperBoneThetaMatList = []

        # to start simple,
        self.referenceBonesList = referenceBonesList

        # Learner's learning parameters
        self.motionLearnerParms = config.MotionLearnerParms()

        self.finalError = 0
        self.finalIteration = 0




    def processStart(self):
        trainingMat = self.createTrainingDataMatrix()
        print '\n\n------ Created TrainingDataMatrix -------'
        for t in trainingMat:
            print t

        # trainingMat, rangeList, avgList = self.normalizeMatrix(trainingMat)
        # print '\n\n------ Nomarlized TrainingDataMatrix -------'
        # for t in trainingMat:
        #     print t
        #
        # print '\n\n---- rangeList ---'
        # print rangeList
        #
        # print '\n\n--- avgList ---'
        # print avgList

        trainingMat = self.addBiasItermToTraningMatrix(trainingMat)
        print '------ Biased TrainingDataMatrix -------'
        for t in trainingMat:
            print t

        self.helperBoneThetaMatList = []

        startTime = time.time()

        for bn in self.helperBonesList:
            labelMat = self.createLabelMatrix(bn)
            print '\n\n--- label matrix ---'
            for l in labelMat:
                print l

            print '\n\n--- learn started --- '
            thetaMat = self.learnTheta(trainingMat, labelMat)
            print '\n\n--- learn finished --- '
            for theta in thetaMat:
                print theta
            self.helperBoneThetaMatList.append(thetaMat)

        for b in range(0, len(self.helperBonesList)):
            bn = self.helperBonesList[b]
            self.util.clearUserProp(bn)
            self.setThetaMatrixToBone(bn, self.helperBoneThetaMatList[b])

        deltaTime = time.time() - startTime

        print 'time costed------> ', deltaTime

    def learnTheta(self, trainingDataMatrix, labelMatrix):
        thetaMat = self.initThetaMatrix(trainingDataMatrix)
        thetaMatTranspose = self.math1.transposeMat(thetaMat)
        print '\n\n--- theta matrix ---'
        for t in thetaMat:
            print t

        sampleCount = len(trainingDataMatrix)
        ratio = self.motionLearnerParms.learningRatio

        lastError = 10000000000
        for i in range(0, self.option.motionLearningIteration):
            # print 'i ', i
            # gradient on each dimension(rotX, rotY, rotZ, transX, transY, transZ)
            for newFeatureIndex in range(0, len(thetaMat)):
                thetaVector = thetaMat[newFeatureIndex]
                # print 'thetaVec ', thetaVec

                predictVector = self.math1.matrixMultiVector(trainingDataMatrix, thetaVector)
                # print 'predictList ', predictList

                labelSampleColumn = self.math1.transposeMat(labelMatrix)[newFeatureIndex]
                # print 'labelSampleColumn ', labelSampleColumn

                deltaColmn = self.math1.vectorSub(predictVector, labelSampleColumn)
                # print 'deltaColmn ', deltaColmn

                for t in range(0, len(thetaVector)):
                    trainingSampleColumn = self.math1.transposeMat(trainingDataMatrix)[t]

                    sumVal = sum(self.math1.vectorElementMulti(deltaColmn,  trainingSampleColumn))
                    # print 'sumVal ', sumVal

                    gdStep = ratio * sumVal / float(sampleCount)
                    # print 'gdStep ', gdStep

                    thetaMat[newFeatureIndex][t] = thetaMat[newFeatureIndex][t] - gdStep

                # print 'gdStep ', gdStep
                # print 'thetaMat[row] ', thetaMat[newFeatureIndex]


            predictMatrix = self.math1.matrixMultiMatrix(trainingDataMatrix, thetaMat)
            # print 'predictMatrix ', predictMatrix
            # print 'labelMatrix ', labelMatrix
            error = self.computeUtil.computeErrorBetweenMatrix(labelMatrix, predictMatrix)

            if lastError - error <= 0.0 or i == self.option.motionLearningIteration - 1:
                self.finalError = error
                self.finalIteration = i
                break
            lastError = error

        print '\n\n--- Learning Finished---'
        print 'learn theta error ', 100.0 * self.finalError / float(self.option.MotionLearningframes)
        print 'learn iteration, ', self.finalIteration
        return thetaMat

    # create the Theta matrix that is 3 x polynom Count
    def initThetaMatrix(self, trainingData):
        polynomTermsCount = len(trainingData[0])
        thetaMat =  [[0.5 for r in range(polynomTermsCount)] for newFeature in range(6)]

        return thetaMat

    def createLabelMatrix(self, bone):
        labelMatrix = []
        for f in range(0, self.option.MotionLearningframes):
            rt.sliderTime = f
            boneLocalXF = bone.Transform
            if bone.parent:
                boneLocalXF = boneLocalXF * rt.inverse(bone.parent.Transform)
            t = boneLocalXF.translation
            print t
            # r =  rt.quatToEuler2(bone.rotation)
            r = self.computeUtil.codeExpotentialMap(bone.rotation)

            # labelMatrix.append([t.x, t.y, t.z])
            labelMatrix.append([t.x, t.y, t.z, r.x, r.y, r.z])
        print 'createLabelMatrix'
        print labelMatrix
        return labelMatrix

    def createTrainingDataMatrix(self):
        trainingDataMat = []
        for f in range(0, self.option.MotionLearningframes):
            rt.sliderTime = f
            polyNomRow = self.rot2Polynom_MulNodes(self.referenceBonesList)
            trainingDataMat.append(polyNomRow)
        return trainingDataMat

    def normalizeMatrix(self, sourceMatrix):
        srcMatTrnsp = self.math1.transposeMat(sourceMatrix)
        rowRangeList = []
        rowAvgList = []
        for r in range(0, len(srcMatTrnsp)):
            rowAvg = sum(srcMatTrnsp[r]) / len(srcMatTrnsp[r])
            rowRange = self.math1.getRange(srcMatTrnsp[r])

            rowRangeList.append(rowRange)
            rowAvgList.append(rowAvg)


            srcMatTrnsp[r] = self.math1.vectorAddFloat(srcMatTrnsp[r], -rowAvg)
            srcMatTrnsp[r] = self.math1.vectorMultiFloat(srcMatTrnsp[r], 1.0/(rowRange + 0.000000000001))

        outMatrix = self.math1.transposeMat(srcMatTrnsp)

        return outMatrix, rowRangeList, rowAvgList

    def addBiasItermToTraningMatrix(self, trainingMat):
        outTrainingMat = []
        for r in range(0, len(trainingMat)):
            newRow = [1] + trainingMat[r]
            outTrainingMat.append(newRow)
        return outTrainingMat

    def rot2Polynom_MulNodes(self, nodeList):
        polyNomMulNode = []
        for nd in nodeList:
            polyNom = self.rot2Polynom(nd)
            polyNomMulNode += polyNom

        return polyNomMulNode

    def rot2Polynom(self, node):
        if node.parent:
            nodeLocalXF = node.Transform * rt.inverse(node.parent.Transform)
        else:
            nodeLocalXF = node.Transform
        # eulerRotation = rt.quatToEuler2(node.Transform.rotation)
        eulerRotation = rt.quatToEuler2(nodeLocalXF.rotation)
        # x = eulerRotation.x / 180.0
        # y = eulerRotation.y / 180.0
        # z = eulerRotation.z / 180.0

        eulerRotation = self.computeUtil.codeExpotentialMap(nodeLocalXF.rotation)

        x = eulerRotation.x
        y = eulerRotation.y
        z = eulerRotation.z
        # return [x, y, z, x*y, x*z, y*z, x*x, y*y, z*z, x*x*x, y*y*y, z*z*z, x*y*z, x*x*y,x*x*z,y*y*x,y*y*z,z*z*x,z*z*y]
        return [x, y, z, x*y, x*z, y*z, x*x, y*y, z*z]#, x*x*x, y*y*y, z*z*z, x*y*z, x*x*y,x*x*z,y*y*x,y*y*z,z*z*x,z*z*y]
        # return [x, y, z]



    def setThetaMatrixToBone(self, bone, thetaMatrix):
        # set the theta matrix to the bone's position and rotation script controller in 3DS Max
        thetaName = ['thetaPosX','thetaPosY','thetaPosZ', 'thetaRotX','thetaRotY','thetaRotZ']
        for t in range(len(thetaMatrix)):
            propString = self.util.convertList_Python2Max(thetaMatrix[t])
            rt.setUserProp(bone, thetaName[t], propString)
