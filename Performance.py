import  time
class PerformanceEvaluator:
    def __init__(self):
        self.allVertIteration = []
        self.allErrorLog = []
        # note the errors infomation is NOT the same as the field finalError in the learner,
        # # as the finalLearningError is the ACCUMULATED error value as the learning prcoess.
        # This is for the performance monitor / evaluation only. Should never been used back into the calculation
        self.finalLearningIteration = 0
        self.finalLearningError = 0
        self.averageLearningIteration = 0
        self.averageLearningError = 0
        self.startTime = 0
        self.finishedTime = 0
        self.consumedTime = 0
        self.startTime = 0
        self.deltaTime = 0

    def logStart(self):
        print '\n\n--- EVALUATION START ---'
        self.startTime = time.time()

    def logResult(self):
        print '\n\n--- EVALUATION RESULT ---'
        print ("Task Finished in + " + str(time.time() - self.startTime) + 'second')
        print ("finalLearningIteration, finalLearningError", self.finalLearningIteration, self.finalLearningError)

        print '\n\n--- EVALUATION RESULT ---'
        for i in range(0, len(self.allErrorLog)):
            print self.allErrorLog[i]

        return

    def newIteration(self):
        self.finalLearningIteration += 1

    def newError(self, error, errorType):
        self.allErrorLog.append([self.finalLearningIteration, errorType, error * 100.0])
