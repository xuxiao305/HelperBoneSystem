helperBones = for h in helpers where (matchpattern h.name pattern:"HP_*") collect h
helperBonesCtrl = #()

maxOps.cloneNodes helperBones cloneType:#copy newNodes:&helperBonesCtrl #nodialog
for h = 1 to helperBonesCtrl.count do 
(
	helperBonesCtrl[h].wirecolor = red
	helperBonesCtrl[h].name = "Ctrl_" + helperBones[h].Name
)

fn SetScriptController controllerType thetaVector =
(
	local controller = undefined
	if controllerType == "position" then
	(	
		controller = position_script()
	)
	else if controllerType == "rotation" then
	(
		controller = rotation_script()
	)
	
	controller.AddConstant  "tx" thetaVector[1]
	controller.AddConstant  "ty" thetaVector[2]
	controller.AddConstant  "tz" thetaVector[3]
	
-- 	controller.AddNode  "PrimBn" ($'Bip001 L Forearm')
	controller.AddNode  "PrimBn" ($'primaryBone')
	
	script = ""
	script += "fn vectorDot vectorA vectorB = (\n"
	script += "	local sum = 0\n"
	script += "	for i = 1 to vectorA.count do ( sum += vectorA[i] * vectorB[i])\n"
	script += "	return sum)\n"

	script += "localXF = PrimBn.Transform * inverse(PrimBn.parent.Transform)\n"
	
-- 	script += "euler2 = quatToEuler2(localXF.rotation)\n"
-- 	script += "x = euler2.x / 180.0\n"
-- 	script += "y = euler2.y / 180.0\n"
-- 	script += "z = euler2.z / 180.0\n"
	
	script += "logQuat = logN(localXF.rotation)\n"
	script += "x = logQuat.x \n"
	script += "y = logQuat.y \n"
	script += "z = logQuat.z \n"

	script += "permutation = #(x, y, z, x*y, x*z, y*z, x*x, y*y, z*z)\n"

	script += "permutation = #(1) + permutation\n"
	script += "xx = vectorDot permutation tx\n"
	script += "yy = vectorDot permutation ty\n"
	script += "zz = vectorDot permutation tz\n"

	if controllerType == "position" then
	(
		script += "localTranslation = point3 xx yy zz\n"

		script += "localTranslation"
	)
	else if controllerType == "rotation" then
	(
		script += "localRotationLog = quat -xx -yy -zz 0 \n"
		script += "localRotation = exp localRotationLog  \n"
		script += "localRotation"
	)
	
	controller.script = script
	
	return controller
)

for HPCtrl in helperBonesCtrl do
(

	local thetaPosX = execute (getUserProp HPCtrl "thetaPosX")
	local thetaPosY = execute (getUserProp HPCtrl "thetaPosY")
	local thetaPosZ = execute (getUserProp HPCtrl "thetaPosZ")
	
	
	local thetaRotX = execute (getUserProp HPCtrl "thetaRotX")
	local thetaRotY = execute (getUserProp HPCtrl "thetaRotY")
	local thetaRotZ = execute (getUserProp HPCtrl "thetaRotZ")
	
	local posScriptCtrl = SetScriptController "position" #(thetaPosX, thetaPosY, thetaPosZ)
	HPCtrl.position.controller = posScriptCtrl
		
	local rotScriptCtrl = SetScriptController "rotation" #(thetaRotX, thetaRotY, thetaRotZ)
	HPCtrl.rotation.controller = rotScriptCtrl	
)

select helperBonesCtrl

macros.run "Animation Tools" "DeleteSelectedAnimation"
