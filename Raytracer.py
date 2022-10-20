from gl import Raytracer, V3
from texture import *
from figures import *
from lights import *


width = 512
height = 512

rtx = Raytracer(width, height)

# env map

rtx.envMap = Texture("oculus.bmp")


# Materiales

stone = Material(diffuse = (0.4, 0.4, 0.4), spec = 8)
yellow = Material(diffuse = (1.2, 2.5, 0.0), spec = 64)
blockstone = Material(spec = 64, texture = Texture("blockstone.bmp"), matType= REFLECTIVE)
wood = Material(spec = 64, texture = Texture("wood.bmp"), matType = OPAQUE)
wood2 = Material(spec = 64, texture = Texture("marble.bmp"), matType = OPAQUE)
mirror = Material(diffuse = (0.9, 0.9, 0.9), spec = 64, matType = REFLECTIVE)
glass = Material(diffuse = (0.9, 0.9, 0.9), spec = 64, ior = 1.5, matType = TRANSPARENT)
cube = Material(spec = 64, texture = Texture("cube.bmp"))
snow = Material(spec = 64, texture = Texture("Snow.bmp"), matType = OPAQUE)
snow2 = Material(spec = 64, texture = Texture("snow2.bmp"), matType = OPAQUE)
snow3 = Material(spec = 64, texture = Texture("snow3.bmp"), matType = OPAQUE)
grass = Material(spec = 64, texture = Texture("grass.bmp"), matType = REFLECTIVE)
# lights

rtx.lights.append( AmbientLight(intensity = 0.1 ))
rtx.lights.append( DirectionalLight(direction = (-1,-1,-1), intensity = 0.5 ))
rtx.lights.append( PointLight(point = (-1,-1,0) ))

#shapes
rtx.scene.append( Sphere((0.4,-1.5,-6),0.60, wood))
rtx.scene.append( Sphere((-1.7,0.08,-6.7),1.5, glass))
rtx.scene.append( Sphere((2,3,-8), 0.8 , mirror))
rtx.scene.append( Sphere((2.5,-1.5,-8.8), 0.9 , grass))



rtx.scene.append( AABB(position = (0.75, 0.10 ,-4), size = (0.5, 0.5, 0.5), material = snow))
rtx.scene.append( AABB(position = (0.75, 0.50 ,-4), size = (0.37, 0.37, 0.37), material = snow2))
rtx.scene.append( AABB(position = (0.75, 0.75 ,-4), size = (0.24, 0.24, 0.24), material = snow3))
rtx.scene.append( AABB(position = (0.01, -0.02 ,-3.5), size = (0.1, 0.05, 0.24), material = wood))


rtx.scene.append( Triangle(b = (3 , 4, -5), a = (4, 2,-5), c = (3,0,-5), material = yellow)) # se refleja en una esfera
rtx.scene.append( Triangle(a = (1,1,-8), b = (-2,-2,-8), c = (3,1,-8), material = mirror)) 

rtx.scene.append( Disk(position = (0,-3,-7), radius = 2, normal = (0,1,0), material = marble ))

rtx.glRender()

rtx.glFinish("output.bmp")
