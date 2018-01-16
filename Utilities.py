from PySide import QtGui, QtCore
import MaxPlus as mp
from pymxs import runtime as rt

import math

# A collection of some helper functions
class Utility:
    def __init__(self):
        return

    def findInList(self, valueList, val):
        for i in range(0, len(valueList)):
            if val == valueList[i]:
                return i
        return -1

    def findInNameList(self, nameList, name):
        for i in range(0, len(nameList)):
            nm = nameList[i]
            if nm == name:
                return i
        return -1

    def findInNodeList(self, nodeList, node):
        for i in range(0, len(nodeList)):
            if node.Name == nodeList[i].Name:
                return i

        return -1


    def normalizeListValue(self, sourceList):
        normList = []
        sumVal = sum(sourceList)

        for i in range(0, len(sourceList)):
            normList.append(sourceList[i] / sumVal)

        return normList

    def makeUnique(self, sourceList):
        uniqueList = list(set(sourceList))
        return uniqueList


    # Nodes
    def sortNodesByDistanceTo(self, sourcePos, nodeList):
        bnList = [""] # the bone list that sorted based on the distance to the source position given in parameter
        bnDistanceList = [-1]
        for i in range(0, len(nodeList)):
            bn = nodeList[i]
            dist = rt.length(nodeList[i].transform[3] - sourcePos)

            index = 0
            for k in range(0, len(bnDistanceList)):
                if dist > bnDistanceList[k]:
                    index += 1

            bnList.insert(index, bn)
            bnDistanceList.insert(index, dist)

        bnList = bnList[1:len(bnList)]

        return bnList

    def convertList_Node2Name(self, nodeList):
        nameList = []

        for bn in nodeList:
            nameList.append(bn.Name)

        return nameList

    def convertList_Name2Node(self, nameList):
        nodeList = []
        for nm in nameList:
            nd = rt.getNodeByName(nm)
            if rt.isValidNode(nd):
                nodeList.append(nd)
        return nodeList


    def convertList_Python2Max(self, srcList):
        outString = '#('
        for i in range(len(srcList)):
            outString += str(srcList[i])
            if i < len(srcList) - 1:
                outString += ','
        outString += ')'
        return outString

    def clearUserProp(self, node):
        rt.setUserPropBuffer(node, '')

    def logListValue(self, nameOfList, sourceList, stepValue = 1):
        print '\n\nlog list value --->', nameOfList
        for i in range(0, len(sourceList), stepValue):
            print sourceList[i]

class SkinModUtility:
    def __init__(self):
        self.util = Utility()

    # internal utilities functions
    def getBoneListFromSkinMod(self, skinMod):
        sortedBoneList = []
        boneCount = rt.skinOps.GetNumberBones(skinMod)

        # need to add one here to match the array range in max
        for i in range(1, boneCount + 1):
            n = rt.skinOps.GetBoneName(skinMod, i, 1)
            bn = rt.getNodeByName(n)
            sortedBoneList.append(bn)

        return sortedBoneList

    def getOneVertBoneAndWeight(self, vertexIndex, originalSkinMod):
        outBonesList = []
        outWeightList = []

        vertexBoneCount = rt.skinOps.GetVertexWeightCount(originalSkinMod, vertexIndex+1)

        # if original bones is in the list, then it takes the priority to be added
        for b in range(0, vertexBoneCount):
            vertexWeightBoneID = rt.skinOps.GetVertexWeightBoneID(originalSkinMod, vertexIndex + 1, b + 1)
            boneName = rt.skinOps.GetBoneName(originalSkinMod, vertexWeightBoneID, 0)
            weight = rt.skinOps.GetVertexWeight(originalSkinMod, vertexIndex + 1, b + 1)

            boneNode = rt.getNodeByName(boneName)

            outBonesList.append(boneNode)
            outWeightList.append(weight)

        outBonesList, outWeightList = self.sortBonesByWeight(outBonesList, outWeightList, isReversed=True)

        return outBonesList, outWeightList

    # Get the vert bone name and weight list
    def getAllVertBoneAndWeight(self, skinMod, skinUtil):
        outAllVertBoneList = []
        outAllWeightList = []
        vertexCount = rt.skinOps.GetNumberVertices(skinMod)
        for v in range(0, vertexCount):
            possibleBonesList, weightList = skinUtil.getOneVertBoneAndWeight(v, skinMod)
            outAllVertBoneList.append(possibleBonesList)
            outAllWeightList.append(weightList)

        return outAllVertBoneList, outAllWeightList

    def convertList_BoneNode2ID(self, skinMod, boneList):
        allBones = self.getBoneListFromSkinMod(skinMod)
        boneIDList = []
        for i in range(0, len(boneList)):
            boneID = self.util.findInNodeList(allBones, boneList[i])
            if boneID != -1:
                boneIDList.append(boneID + 1)

        return boneIDList

    def sortBonesByWeight(self, boneList, weightList, isReversed = False):
        sortedBoneList = ['']
        sortedWeightList = [0]

        for w in range(0, len(weightList)):
            index = 0
            for w2 in range(0, len(sortedWeightList)):
                if weightList[w] >= sortedWeightList[w2]:
                    index += 1

            sortedBoneList.insert(index, boneList[w])
            sortedWeightList.insert(index, weightList[w])

        sortedBoneList = sortedBoneList[1: len(sortedBoneList)]
        sortedWeightList = sortedWeightList[1: len(sortedWeightList)]

        if isReversed:
            sortedBoneList.reverse()
            sortedWeightList.reverse()
        return sortedBoneList, sortedWeightList

    def initWeightList(self, possibleBonesList):
        weightInProgress = []

        for i in range(0, len(possibleBonesList)):
            weightInProgress.append(1.0 / len(possibleBonesList))

        return weightInProgress


class ComputingUtility:
    def __init__(self):
        return

    def computeAvgErrorForAllVerts(self, regularAllVertPosList, referenceAllVertPosList, frames):
        totalError = self.computeErrorForAllVerts(regularAllVertPosList, referenceAllVertPosList)
        totalError = totalError / (float(len(regularAllVertPosList)))
        totalError = totalError / (float(frames))

        return totalError

    def computeErrorForAllVerts(self, regularAllVertPosList, referenceAllVertPosList):
        totalError = 0

        for v in range(1, len(regularAllVertPosList)):
            error = self.computeErrorForOneVert(regularAllVertPosList[v], referenceAllVertPosList[v])
            totalError += error


        return totalError


    def findVertWithMaxError(self, regularAllVertPosList, referenceAllVertPosList):
        print 'findVertWithMaxError'
        maxError = 0
        maxErrorVertexIndex = 0
        maxErrorVertPos = rt.Point3(0,0,0)

        for v in range(1, len(regularAllVertPosList)):
            error = self.computeErrorForOneVert(regularAllVertPosList[v], referenceAllVertPosList[v])
            # print 'v, error',v, error
            if error > maxError:
                maxError = error
                maxErrorVertexIndex = v
                maxErrorVertPos = regularAllVertPosList[v][0]
                # print 'maxErrorVertexIndex ', maxErrorVertexIndex
                # print 'maxError ', maxError


        return maxErrorVertexIndex, maxErrorVertPos, maxError

    def computeErrorForOnePos(self, regularPos, referencePos):
        error = rt.length(regularPos - referencePos)
        return error

    def computeErrorForOneVert(self, regularPosList, referencePosList):
        error = 0
        for f in range(0, len(regularPosList)):
            error += rt.length(regularPosList[f] - referencePosList[f])
        return error

    def computeErrorBetweenMatrix(self, matA, matB):
        error = 0
        for i in range(0, len(matA)):
            error += self.computeErrorBetweenVector(matA[i], matB[i])

        return error

    def computeErrorBetweenVector(self, vectorA, vectorB):
        error = 0
        for i in range(0, len(vectorA)):
            error += math.pow(vectorA[i] - vectorB[i], 2.0)

        return error

    def applyTransform(self, position, weight, transform, restTransform):
        # transform the vertex for given weight
        p = position * rt.inverse(restTransform) * transform * weight

        return p

    # the function that simulate the vertex transform in given weight list
    # weightList -- the weights of bones for 1 vertex
    # transformList -- the matrix list for all frames of all bones for 1 vert
    # TODO: it should be faster if i pass the position tuble with the full length, and modify the value, instead of creating a new mutable list
    def TransformPosition(self, position, weightList, transformList, frames):
        outPositionList = []
        for f in range(0, frames):

            # transform the vertex for given frame
            p = rt.Point3(0, 0, 0)
            for w in range(0, len(weightList)):
                p += position * rt.inverse(transformList[w][0]) * transformList[w][f] * weightList[w]

            # and store them in outPositionList
            outPositionList.append(p)

        return outPositionList



    def codeExpotentialMap(self, rotation):
        lnRot = rt.logN (rotation)
        return lnRot

    def decodeExpotentialMap(self, logRotation):
        quat = rt.quat(logRotation[0], logRotation[1], logRotation[2], 0)
        result = math.exp(quat)
        return result


class VertexUtility:
    # Vertex
    def getAvgPosFromVertex(self, object, vertIndexList):
        sumPos = rt.Point3(0, 0, 0)
        for v in vertIndexList:
            sumPos += object.verts[v].pos

        avgPos = sumPos / len(vertIndexList)

        return avgPos

    def sortVertsByDistanceTo(self, sourcePos, vertsList):
        outList = [""]
        vertDistanceList = [-1]
        for i in range(0, len(vertsList)):
            dist = rt.length(vertsList[i].pos - sourcePos)

            index = 0
            for k in range(0, len(vertDistanceList)):
                if dist > vertDistanceList[k]:
                    index += 1

            outList.insert(index, outList[i])
            vertDistanceList.insert(index, dist)

            outList = outList[1:len(outList)]

        return outList

    def convertList_IndexToVertex(self, indexList, obj):
        outVertList = []
        for v in indexList:
            outVertList.append(obj.verts[v])

        outVertList

    def convertList_bit2Array(self, bitArray, oneBased = False):
        outList = []
        for f in range(0, len(bitArray)):
            if bitArray[f]:
                if oneBased:
                    outList.append(f+1)
                else:
                    outList.append(f)
        return outList

    def getNeighborVertex(self, obj, vertIndex):
        # note that the vertex index is sent in as 0-based, but the function getFacesUsingVert is 1 based.
        vertFacesBitArray = (rt.polyOp.getFacesUsingVert(obj, vertIndex + 1))
        vertFacesList = self.convertList_bit2Array(vertFacesBitArray, oneBased = True)

        faceVertsBitArray = rt.polyOp.getVertsUsingFace(obj, vertFacesList)
        faceVertsList = self.convertList_bit2Array(faceVertsBitArray)

        # rt.polyOp.setVertSelection(obj, faceVertsList)

        return faceVertsList




