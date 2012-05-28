import euclid as eu
import copy

OFFSET = 0.15

# coordinates
             # PT 1
MONASTERY = [(1.1, 0.75),
             # PT 2
             (1.2, 1.95),
             # PT 21
             (1.1, 0.75)]

def scaleadd(origin, offset, vectorx):
    """
    From a vector representing the origin,
    a scalar offset, and a vector, returns
    a Vector3 object representing a point 
    offset from the origin.

    (Multiply vectorx by offset and add to origin.)
    """
    multx = vectorx * offset
    return multx + origin

def getinsetpoint(pt1, pt2, pt3):
    """
    Given three points that form a corner (pt1, pt2, pt3),
    returns a point offset distance OFFSET to the right
    of the path formed by pt1-pt2-pt3.

    pt1, pt2, and pt3 are two tuples.

    Returns a Vector3 object.
    """
    origin = eu.Vector3(pt2[0], pt2[1], 0.0)
    v1 = eu.Vector3(pt1[0] - pt2[0], 
                    pt1[1] - pt2[1], 0.0)
    v1.normalize()
    v2 = eu.Vector3(pt3[0] - pt2[0], 
                    pt3[1] - pt2[1], 0.0)
    v2.normalize()
    v3 = copy.copy(v1)
    v1 = v1.cross(v2)
    v3 += v2
    if v1.z < 0.0:
        retval = scaleadd(origin, -OFFSET, v3)
    else:
        retval = scaleadd(origin, OFFSET, v3)
    return retval

polyinset = []
lenpolygon = len(MONASTERY)
i = 0
poly = MONASTERY
while i < lenpolygon - 2:
    polyinset.append(getinsetpoint(poly[i], 
                 poly[i + 1], poly[i + 2]))
    i += 1
polyinset.append(getinsetpoint(poly[-2], 
             poly[0], poly[1]))
polyinset.append(getinsetpoint(poly[0], 
             poly[1], poly[2]))

