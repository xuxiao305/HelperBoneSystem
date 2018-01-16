toDel = #()
for p in helpers do
(
	if (matchpattern p.name pattern:"Point0*") do append toDel p
)


delete toDel
clearListener()
-- disableSceneRedraw()

-- select $Regular
sliderTime = 0
-- if $.modifiers[1].name == "Skin_Learnt" then deleteModifier $ 1
	
max modify mode  
-- p = getThisScriptFilename()
-- p = replace p (p.count-1) 2 "py"

python.ExecuteFile @"D:\OneDrive\ToyLabsP4v\PythonWorks\HelperBones\Main.py"
-- python.ExecuteFile @"C:\Users\xxiao2\OneDrive\ToyLabsP4v\PythonWorks\HelperBones\HelperBonesUI.py"
-- enableSceneRedraw()
-- gc lit:True

savemaxfile (maxfilepath + maxfilename) 