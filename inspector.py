#~ obj = rt.getNodeByName('Point001')
#~ objINode = mp.INode.GetINodeByHandle(obj.handle)
#~ print objINode 


import inspect

classes = inspect.getmembers(rt.position_script)
for c in classes:
	print c
	
	
