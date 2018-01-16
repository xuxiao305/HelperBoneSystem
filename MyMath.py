class SimpleMath:
    def __init__(self):
        return

    def vectorElementMulti(self, vectorA, vectorB):
        if len(vectorA) != len(vectorB):
            print 'WARNING! vectorElementMulti - length of vectors do NOT match!'
            return

        resut = []
        for i in range(0, len(vectorA)):
            resut.append(vectorA[i] * vectorB[i])

        return  resut

    def getMaxVal(self, vectorA):
        maxVal = -10000000
        for v in vectorA:
            if v > maxVal:
                maxVal = v
        return maxVal

    def getMinVal(self, vectorA):
        minVal = 10000000
        for v in vectorA:
            if v < minVal:
                minVal = v
        return minVal

    def getRange(self, vectorA):
        outRange = self.getMaxVal(vectorA) - self.getMinVal(vectorA)
        return outRange

    def vectorAddFloat(self, vectorA, floatB):
        result = [0 for i in range(len(vectorA))]
        for i in range(0, len(result)):
            result[i] = vectorA[i] + floatB
        return result

    def vectorMultiFloat(self, vectorA, floatB):
        result = [0 for i in range(len(vectorA))]
        for i in range(0, len(result)):
            result[i] = vectorA[i] * floatB
        return result

    def vectorAdd(self, vectorA, vectorB):
        result = [0 for i in range(len(vectorA))]
        for i in range(0, len(result)):
            result[i] = vectorA[i] + vectorB[i]

        return result

    def vectorSub(self, vectorA, vectorB):
        result = [0 for i in range(len(vectorA))]
        for i in range(0, len(result)):
            result[i] = vectorA[i] - vectorB[i]

        return result


    def vectorDot(self, vectorA, vectorB):
        tempVec = []
        for i in range(0, len(vectorA)):
            tempVec.append(vectorA[i] * vectorB[i])

        result = sum(tempVec)
        return result

    def matrixMultiVector(self, mat, vec):
        result = []
        for i in range(0, len(mat)):
            result.append(self.vectorDot(mat[i], vec))

        return result

    def matrixMultiMatrix(self, matA, matB):
        result = []
        for a in range(0, len(matA)):
            row = []
            for b in range(0, len(matB)):
                row.append(self.vectorDot(matA[a], matB[b]))

            result.append(row)

        return result

    def matrixGetColumn(self, matA, col):
        tempMat = self.transposeMat(matA)
        return tempMat[col]

    def transposeMat(self, mat):
        result = [[0 for row in range(len(mat))] for col in range(len(mat[0]))]

        for i in range(0, len(result)):
            for j in range(0, len(result[i])):
                result[i][j] = mat[j][i]


        return result

