import sys

def is_odd(x): return x%2 == 1

def rayintersectseg(p, a, b):
    ''' takes a point p=Pt() and an edge of two endpoints a,b=Pt() of a line segment returns boolean
    '''
    _eps = 0.00001
    _huge = sys.float_info.max
    _tiny = sys.float_info.min

    if a[1] > b[1]:
        a,b = b,a
    if p[1] == a[1] or p[1] == b:
        p = [p[0], p[1] + _eps  ]
 
    intersect = False
 
    if (p[1] > b[1] or p[1] < a[1]) or (p[0] > max(a[0], b[0])):
        return False
 
    if p[0] < min(a[0], b[0]):
        intersect = True
    else:
        if abs(a[0] - b[0]) > _tiny:
            m_red = (b[1] - a[1]) / float(b[0] - a[0])
        else:
            m_red = _huge
        if abs(a[0] - p[0]) > _tiny:
            m_blue = (p[1] - a[1]) / float(p[0] - a[0])
        else:
            m_blue = _huge
        intersect = m_blue >= m_red
    return intersect



def ispointinside(p, poly):
    ln = len(poly)
    count=0
    for i in range(ln-1):
        Ax=poly[i][0]
        Bx=poly[i+1][0]
        Ay=poly[i][1]
        By=poly[i+1][1]
        A=[Ax,Ay]
        B=[Bx,By]

        if rayintersectseg(p, A, B):
            count+=1

    return not is_odd(count)