import scipy.interpolate
import numpy as np

knots = [0, 1, 2, 3]
knots = [knots[0], knots[0]] + knots + [knots[-1], knots[-1]]
knots = [knots[0] * 2 - knots[1]] + knots + [knots[-1] * 2 - knots[-2]]
knots = [knots[0] * 2 - knots[1]] + knots + [knots[-1] * 2 - knots[-2]]
points = [(1,1), (2,1), (2,2), (3,2)]
points = [points[0], points[0]] + points + [points[-1], points[-1]]

spline =  scipy.interpolate.BSpline(knots, points, 3)
out = spline(np.arange(0,3,0.1))