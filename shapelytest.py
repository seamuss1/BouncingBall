import shapely.geometry as geo
import shapely.ops
import matplotlib.pyplot as plt
box = geo.box(-30,0,30,50)

p = geo.point.Point(29.5,25)
circle = p.buffer(1)

res = shapely.ops.snap(circle,box,0.6)

x,y = circle.exterior.xy
x2,y2 = box.exterior.xy
x3,y3 = res.exterior.xy
##plt.plot(x,y,x2,y2)
##plt.show()

dis = shapely.ops.nearest_points(p,box)[1]
##print(dis)

print(box.project(p))
