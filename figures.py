import numpy as np

WHITE = (1,1,1)
BLACK = (0,0,0)

OPAQUE = 0
REFLECTIVE = 1
TRANSPARENT = 2


class Intersect(object):
    def __init__(self, distance, point, normal, texcoords, sceneObj):
        self.distance = distance
        self.point = point
        self.normal = normal
        self.texcoords = texcoords
        self.sceneObj = sceneObj

class Material(object):
    def __init__(self, diffuse = WHITE, spec = 1.0, ior = 1.0, texture = None, matType = OPAQUE):
        self.diffuse = diffuse
        self.spec = spec
        self.ior = ior
        self.texture = texture
        self.matType = matType


class Sphere(object):
    def __init__(self, center, radius, material):
        self.center = center
        self.radius = radius
        self.material = material

    def ray_intersect(self, orig, dir):
        L = np.subtract(self.center, orig)
        tca = np.dot(L, dir)
        d = (np.linalg.norm(L) ** 2 - tca ** 2) ** 0.5

        if d > self.radius:
            return None

        thc = (self.radius ** 2 - d ** 2) ** 0.5

        t0 = tca - thc
        t1 = tca + thc

        if t0 < 0:
            t0 = t1
        if t0 < 0:
            return None
        
        # P = O + t0 * D
        P = np.add(orig, t0 * np.array(dir))
        normal = np.subtract(P, self.center)
        normal = normal / np.linalg.norm(normal)

        u = 1 - ((np.arctan2(normal[2], normal[0]) / (2 * np.pi)) + 0.5)
        v = np.arccos(-normal[1]) / np.pi

        uvs = (u,v)

        return Intersect(distance = t0,
                         point = P,
                         normal = normal,
                         texcoords = uvs,
                         sceneObj = self)


class Plane(object):
    def __init__(self, position, normal,  material):
        self.position = position
        self.normal = normal / np.linalg.norm(normal)
        self.material = material

    def ray_intersect(self, orig, dir):
        # Distancia = (( planePos - origRayo) o normal) / (direccionRayo o normal)
        denom = np.dot( dir, self.normal)

        if abs(denom) > 0.0001:
            num = np.dot( np.subtract(self.position, orig), self.normal)
            t = num / denom

            if t > 0:
                # P = O + t*D
                P = np.add(orig, t * np.array(dir))
                return Intersect(distance = t,
                                 point = P,
                                 normal = self.normal,
                                 texcoords = None,
                                 sceneObj = self)

        return None

class Disk(object):
    def __init__(self, position, radius, normal,  material):
        self.plane = Plane(position, normal, material)
        self.material = material
        self.radius = radius

    def ray_intersect(self, orig, dir):

        intersect = self.plane.ray_intersect(orig, dir)

        if intersect is None:
            return None

        contact = np.subtract(intersect.point, self.plane.position)
        contact = np.linalg.norm(contact)

        if contact > self.radius:
            return None

        return Intersect(distance = intersect.distance,
                         point = intersect.point,
                         normal = self.plane.normal,
                         texcoords = None,
                         sceneObj = self)

# Referencias ejecución de triangulo 
#https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
# https://docs.pyvista.org/examples/99-advanced/ray-trace-moeller.html codigo compartido en el discord
#https://www.erikrotteveel.com/python/three-dimensional-ray-tracing-in-python/


class Triangle(object):
    # break down triangle into the individual points
    def __init__(self, a, b, c, material):
        self.a = a #v1
        self.b = b #v2
        self.c = c #v3
        self.material = material
        
# compute edges
    #edge1 = v2 - v1
    #edge2 = v3 - v1
    #pvec = np.cross(ray_vec, edge2)
    #det = edge1.dot(pvec)
    def ray_intersect(self, orig, dir): #def intersectionRayModel(self, rayStart, rayEnd):
        edge1 = np.subtract(self.b, self.a)
        edge2 = np.subtract(self.c, self.a)
        eps = -0.000001
        epsi = 0.000001
        
        normal = np.cross(edge1, edge2)
        
        #pvec = np.cross(ray_vec, edge2)
        #det = edge1.dot(pvec)
        
        pvec = np.cross(dir, edge2)
        det = edge1.dot(pvec)
       
        
        if abs(det) < eps and det < epsi:
            return None
        inv_det = 1.0 / det
        tvec = np.subtract(orig,self.a)
        u = tvec.dot(pvec) * inv_det
        if u < 0.0 or u > 1.0:  # if not intersection
            return None

        qvec = np.cross(tvec, edge1)
        v = np.dot(dir, qvec)*inv_det
        if v < 0.0 or u + v > 1.0:  # if not intersection
            return None 

        t = edge2.dot(qvec) * inv_det
        if t > 0.000001:
            pointo = np.add(orig, t * np.array(dir) )
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal) 
            return Intersect( distance = t,
                              point = pointo,
                              normal = normal,
                              texcoords = (u, v),
                              sceneObj = self)

        else:
            return None
   
        
        

class AABB(object):
    # Axis Aligned Bounding Box

    def __init__(self, position, size, material):
        self.position = position
        self.size = size
        self.material = material

        self.planes = []

        halfSizes = [0,0,0]

        halfSizes[0] = size[0] / 2
        halfSizes[1] = size[1] / 2
        halfSizes[2] = size[2] / 2

        # Sides
        self.planes.append( Plane( np.add(position, (halfSizes[0],0,0)), (1,0,0), material ))
        self.planes.append( Plane( np.add(position, (-halfSizes[0],0,0)), (-1,0,0), material ))

        # Up and Down
        self.planes.append( Plane( np.add(position, (0,halfSizes[1],0)), (0,1,0), material ))
        self.planes.append( Plane( np.add(position, (0,-halfSizes[1],0)), (0,-1,0), material ))

        # Front and back
        self.planes.append( Plane( np.add(position, (0,0,halfSizes[2])), (0,0,1), material ))
        self.planes.append( Plane( np.add(position, (0,0,-halfSizes[2])), (0,0,-1), material ))

        #Bounds
        self.boundsMin = [0,0,0]
        self.boundsMax = [0,0,0]

        epsilon = 0.001

        for i in range(3):
            self.boundsMin[i] = self.position[i] - (epsilon + halfSizes[i])
            self.boundsMax[i] = self.position[i] + (epsilon + halfSizes[i])


    def ray_intersect(self, orig, dir):
        intersect = None
        t = float('inf')

        for plane in self.planes:
            planeInter = plane.ray_intersect(orig, dir)
            if planeInter is not None:

                planePoint = planeInter.point

                if self.boundsMin[0] <= planePoint[0] <= self.boundsMax[0]:
                    if self.boundsMin[1] <= planePoint[1] <= self.boundsMax[1]:
                        if self.boundsMin[2] <= planePoint[2] <= self.boundsMax[2]:

                            if planeInter.distance < t:
                                t = planeInter.distance
                                intersect = planeInter

                                # Tex Coords

                                u, v = 0, 0

                                # Las uvs de las caras de los lados
                                if abs(plane.normal[0]) > 0:
                                    # Mapear uvs para el eje x, usando las coordenadas de Y y Z
                                    u = (planeInter.point[1] - self.boundsMin[1]) / self.size[1]
                                    v = (planeInter.point[2] - self.boundsMin[2]) / self.size[2]

                                elif abs(plane.normal[1] > 0):
                                    # Mapear uvs para el eje y, usando las coordenadas de X y Z
                                    u = (planeInter.point[0] - self.boundsMin[0]) / self.size[0]
                                    v = (planeInter.point[2] - self.boundsMin[2]) / self.size[2]

                                elif abs(plane.normal[2] > 0):
                                    # Mapear uvs para el eje z, usando las coordenadas de X y Y
                                    u = (planeInter.point[0] - self.boundsMin[0]) / self.size[0]
                                    v = (planeInter.point[1] - self.boundsMin[1]) / self.size[1]


        if intersect is None:
            return None

        return Intersect(distance = t,
                         point = intersect.point,
                         normal = intersect.normal,
                         texcoords = (u,v),
                         sceneObj = self)

