import MaxPlus as mp
from pymxs import runtime as rt

class ConstraintSetter:
    def __init__(self):
        return

    def assignPositionScriptConstraint(self, obj, constantNameList, constantValueList, sourceNodeList, scripts):
        posScriptConstraint = rt.position_script()
        posScriptConstraint = mp.Factory.CreatePositionController()

        objINode = mp.INode.GetINodeByHandle(obj.handle)

        for i in range(len(constantNameList)):
            posScriptConstraint.AddConstant (constantNameList[i], constantValueList[i])

        objTMController = objINode.GetTMController()
        objTMController.SetPositionController(posScriptConstraint)

        sourceINodeList = []
        # for srcNode in sourceNodeList:
        #     newSrcINode = mp.INode.GetINodeByHandle(srcNode.handle)
        #     sourceINodeList.append(newSrcINode)
        #     objTMController.AddObject (srcNode.name, newSrcINode)



constraintSetter = ConstraintSetter()
obj = rt.selection[0]
constantNameList = ['a','b']
constantValueList = [1,2]
sourceNodeList = None
script = ''
constraintSetter.assignPositionScriptConstraint(obj, constantNameList, constantValueList, sourceNodeList, script)


