class SystemOption:
    def __init__(self):
        self.sampleStep = 1
        self.frames = 200
        self.MotionLearningframes = 200
        self.boneLimitation = 8
        self.maxHelperBoneCount = 6

        self.iteration = 60
        self.motionLearningIteration = 100000

        self.learntSkinModName = 'Skin_Learnt'

        self.skinningParms = PoseLearnerParms()

        self.translateParms = PoseLearnerParms()
        self.translateParms.adaptiveStepRange = (0.01, 0.2)

        self.rotateParms = PoseLearnerParms()
        self.rotateParms.adaptiveStepRange = (0.01, 0.2)

        self.tolerantError = 0.0001

        self.helperBonePrefix = "HP_"
        self.ControlledHelperBonePrefix = "HPCtrl_"
        # temp variables
        # self.primaryBoneNameList = ['Bip001 L Clavicle', 'Bip001 L UpperArm', 'Bip001 L Forearm']
        self.primaryBoneNameList = ['primaryBone']
        # self.tempReferenceBoneList = ['Bip001 L Forearm']
        self.tempReferenceBoneList = ['primaryBone']
        self.regularMeshName = 'Regular'
        self.referenceMeshName = 'Reference'


class PoseLearnerParms:
    def __init__(self,
                 step = 0.1,
                 adaptiveStepRange = (0.05, 0.2),
                 adaptiveStep = True,
                 adaptiveRange = (0.01, 0.2)):

        self.step = step
        self.adaptiveStepRange = adaptiveStepRange
        self.adaptiveStep = adaptiveStep
        self.adaptiveRange = adaptiveRange

    def getAdaptiveStep(self, error):
        adaptiveStep = (error - self.adaptiveRange[0]) / (self.adaptiveRange[1] - self.adaptiveRange[0])

        adaptiveStep = max(self.adaptiveStepRange[0], adaptiveStep)
        adaptiveStep = min(self.adaptiveStepRange[1], adaptiveStep)

        return adaptiveStep


class MotionLearnerParms:
    def __init__(self, learningRatio = 0.005):
        self.learningRatio = learningRatio
