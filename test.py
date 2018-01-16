import MaxPlus as mp
from pymxs import runtime as rt

class ConstraintSetter:
    def __init__(self):
        return

    def assignPositionScriptConstraint(self, obj, constantNameList, constantValueList, targetNodeList, scripts):
        posScriptConstraint = mp.Factory.CreatePositionController(mp.ClassIds.position_script)
        print posScriptConstraint.Script
        objINode = mp.INode.GetINodeByHandle(obj.handle)

        for nd in targetNodeList:
            targetINode = mp.INode.GetINodeByHandle(nd.handle)
            posScriptConstraint.SetTarget (targetINode)

        objTMController = objINode.GetTMController()
        print (type(objTMController))
        objTMController.SetPositionController(posScriptConstraint)

        sourceINodeList = []
        # for srcNode in targetNodeList:
        #     newSrcINode = mp.INode.GetINodeByHandle(srcNode.handle)
        #     sourceINodeList.append(newSrcINode)
        #     objTMController.AddObject (srcNode.name, newSrcINode)



constraintSetter = ConstraintSetter()
obj = rt.getNodeByName('Point001')
constantNameList = ['a','b']
constantValueList = [1,2]
sourceNodeList = [rt.getNodeByName('target')]
script = ''
constraintSetter.assignPositionScriptConstraint(obj, constantNameList, constantValueList, sourceNodeList, script)