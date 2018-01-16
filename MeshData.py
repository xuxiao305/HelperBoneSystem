from Utilities import *
from SystemOptions import *
from pymxs import runtime as rt
import MaxPlus as mp

class ReferenceMeshData:
    def __init__(self, referenceMesh, systemOption):
        self.util = Utility()
        self.systemOptions = systemOption
        self.obj = referenceMesh
        self.allVertPosList = []

        self.initData()

    def initData(self):
        self.allVertPosList = [[0 for t in range(self.systemOptions.frames)] for v in range(len(self.obj.verts))]
        for f in range(0, self.systemOptions.frames):
            rt.sliderTime = f * self.systemOptions.sampleStep
            for v in range(0, len(self.obj.verts)):
                self.allVertPosList[v][f] = self.obj.verts[v].pos

class RegularMeshData:
    def __init__(self, regularMesh, primarySkinMod, primaryBoneNameList, systemOption):
        self.systemOptions = systemOption
        self.util = Utility()
        self.skinModUtil = SkinModUtility()
        self.obj = regularMesh

        # The original, skin modifier without helper bone.
        self.primarySkinMod = primarySkinMod


        ############ The primary and helper bones list ############
        self.primaryBoneNameList = primaryBoneNameList
        self.primaryBoneList = self.util.convertList_Name2Node(self.primaryBoneNameList)

        self.helperBoneList = []

        # Store all the bones used in the system
        self.allBoneList = self.primaryBoneList + self.helperBoneList

        ############ the Data of Vertex, Bone Name, stored in their own list #############
        # Store the vertex position. The value inside will be transformed by the bonesXFList
        self.allVertPosList = []

        # A 2D list store the skinning bone of each vertex in the following format:
        # Row = vertex. Column = bones
        # The row length should = length of vertex, and the length columns vary from one vertex to another
        self.allVertBoneList = []

        # A 2D list store the weight of each vertex for each bone as in following format:
        # Row = vertex. Column = weight for each bone
        # The row length should = length of vertex, and the length columns vary from one vertex to another
        self.allWeightList = []


        # A sublist of allVertBoneList, just for a easy access to getting new possible bone
        self.allVertPrimaryBoneList = []

        # A 2D list store the transform (pose) of each bone in each frame as in following format
        # row = each bone, column = frame index.
        # Note the order of row is the same with allBoneList
        self.allBonesXFList = []


        #################################################################################################

        self.initData(primarySkinMod)


    ################ The Reader Functions that Read Data from the Scene ################
    def initData(self, skinMod):
        print "--- INIT DATA ---"


        # Get the vertex position
        def initAllVertPosList():
            outAllVertPosList = [[0 for i in range(self.systemOptions.frames)] for v in range(len(self.obj.verts))]
            for f in range(0, self.systemOptions.frames):
                rt.sliderTime = f * self.systemOptions.sampleStep
                for v in range(0, len(self.obj.verts)):
                    outAllVertPosList[v][f] = self.obj.verts[v].pos
            return outAllVertPosList



        def initAllBoneXF():
            outAllBonesXFList = [[0 for t in range(self.systemOptions.frames)] for v in
                                 range(len(self.allBoneList))]

            for f in range(0, self.systemOptions.frames):
                rt.sliderTime = f * self.systemOptions.sampleStep
                for b in range(0, len(self.allBoneList)):
                    outAllBonesXFList[b][f] = self.allBoneList[b].transform

            return outAllBonesXFList

        self.primarySkinMod.enabled = True
        self.allVertPosList = initAllVertPosList()
        self.primarySkinMod.enabled = False

        self.allVertBoneList, self.allWeightList = self.skinModUtil.getAllVertBoneAndWeight(skinMod, self.skinModUtil)
        # In the initialization phase, the there is no helper bone!
        self.allVertPrimaryBoneList = self.allVertBoneList

        self.allBonesXFList = initAllBoneXF()
        # self.util.logListValue('regular vert pos list, row for frame, column for vertex', self.allVertPosList)


    ################ The Helper Functions that Help Updating the Iterative Data from the Learner ################
    # Add the helper bone into the helper bone list, and update the vertex weight
    def includeNewBone(self, newBone, coreNeighborVerts, outerNeighborVerts):
        print "--- ADD NEW HELPER BONE ---"
        self.helperBoneList.append(newBone)
        self.allBoneList.append(newBone)
        newBoneXF = [newBone.Transform for t in range(self.systemOptions.frames)]
        self.allBonesXFList.append(newBoneXF)

        for v in (coreNeighborVerts + outerNeighborVerts):
            self.allVertBoneList[v].append(newBone)

        for v in coreNeighborVerts:
            # self.allWeightList[v] = [0 for i in range(len(self.allWeightList[v]))]
            self.allWeightList[v].append(1)
            self.allWeightList[v] = self.util.normalizeListValue(self.allWeightList[v])

        for v in outerNeighborVerts:
            self.allWeightList[v].append(0.2)
            self.allWeightList[v] = self.util.normalizeListValue(self.allWeightList[v])


    # Refresh possible bone list
    # # Finding the vertices that added the new bone into their vertBoneList, then reset the weight equally
    # # If not, then the vertex keep its original weight
    def updateAllVertBoneList(self):
        def getPossibleBoneNWeightForOneVert(vertexIndex):
            vertex = self.obj.verts[vertexIndex]
            currentPrimBoneList = self.allVertPrimaryBoneList[vertexIndex]
            bonesListByDistance = self.util.sortNodesByDistanceTo(vertex.pos, self.helperBoneList)

            # Note sysOption.boneLimitation is different than the sysOption.maxHelperBoneCount
            vertMaxHelperBoneCount = self.systemOptions.boneLimitation - len(currentPrimBoneList)
            newVertHelperBoneList = bonesListByDistance[0 : vertMaxHelperBoneCount]
            # The existed primary bone join first, then add helper bone to join until it reach the bone limitation for CURRENT vertex
            newVertBonesList = currentPrimBoneList + newVertHelperBoneList

            # If the new bone list is different, then reset the weight.
            oldVertBonesList = self.allVertBoneList[vertexIndex]
            if newVertBonesList != oldVertBonesList:
                newWeightList = self.allWeightList[vertexIndex] + [0]
                return newVertBonesList, newWeightList

            return None, None

        for v in range(0, len(self.allVertBoneList)):
            curPossibleBonesList, curInitWeight = getPossibleBoneNWeightForOneVert(v)
            if curPossibleBonesList:
                self.allVertBoneList[v] = curPossibleBonesList
                self.allWeightList[v] = curInitWeight

    # Remove the helper bone with too little influence on the mesh, and normalize the weight for the remaining bones
    def removeHelperBone(self):
        return

    def queryMutilVertBoneTransformList(self, bnNodeList):
        multiQueriedXFList = []
        for bn in bnNodeList:
            multiQueriedXFList.append(self.queryBoneTransformList(bn))

        return multiQueriedXFList

    # the function that query the matrix list of certain bone in the field allBonesXFList
    def queryBoneTransformList(self, bnNode):
        for b in range(0, len(self.allBoneList)):
            if self.allBoneList[b].Name == bnNode.Name:
                queriedXFList = self.allBonesXFList[b]
                return queriedXFList

        return None

    # get the vertices that influenced by give bone
    def getVertIndexByBone(self, boneNode):
        outVerticesList = []

        for v in range(0, len(self.allVertBoneList)):
            for bn in self.allVertBoneList[v]:
                if bn.Name == boneNode.Name:
                    outVerticesList.append(v)
        return outVerticesList


    ################ The Setter Functions that Set the Learnt Data Back to Scene ################
    # update a single bone xf with the new transform of the bones
    def setLearntSkinMod(self):
        print '--- SET LEARNT SKIN MODIFIER ---'
        self.util = Utility()
        newSkinMod = rt.Skin()
        newSkinMod.Name = self.systemOptions.learntSkinModName

        for bn in self.allBoneList:
            rt.skinOps.addBone(newSkinMod, bn, 0)

        rt.AddModifier(self.obj, newSkinMod)

        for v in range(0, len(self.obj.verts)):
            bnList = self.allVertBoneList[v]
            boneIDList = self.skinModUtil.convertList_BoneNode2ID(newSkinMod, bnList)
            weightList = self.allWeightList[v]
            rt.skinOps.ReplaceVertexWeights(newSkinMod, v + 1, boneIDList, weightList)

        return newSkinMod

    def setLearntBoneXF(self):
        print '--- SET LEARNT TRANSFORM ---'

        for f in range(0, self.systemOptions.frames):
            t = f * self.systemOptions.sampleStep
            rt.sliderTime = t
            for b in range(0, len(self.allBoneList)):
                bn = self.allBoneList[b]
                if self.util.findInNodeList(self.helperBoneList, bn) != -1:
                    bn = self.allBoneList[b]
                    bn.Transform = self.allBonesXFList[b][f]

        rt.sliderTime = 0


