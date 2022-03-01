from klampt import vis
import klampt
world = klampt.WorldModel()
robot= world.robot(0)
link = robot.link(2)
world.loadElement("data/robots/planar3R.rob")
vis.add("world",world)    #shows the robot in the solved configuration
vis.add("local point",link.getWorldPosition([1,0,0]))
vis.setAttribute("local point","type","Vector3")  #usually the vis module identifies a Config vs a Vector3, but this robot has exactly 3 links
vis.add("target point",[1.5,0,1])
vis.setAttribute("target point","type","Vector3")
vis.setColor("target point",1,0,0)  #turns the target point red
vis.show()  #this will pop up the visualization window until you close it