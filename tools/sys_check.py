print('system imports..')

print('math imports..')
import mdp
import numpy

print('image imports..')
from PIL import Image

print('OpenGL imports..')
from OpenGL.GL import *

print('Everything seems to be in order.')
print('Versions you are using:')
print('MDP:    ', mdp.__version__)
print('Numpy:  ', numpy.__version__)
print('PIL:    ', Image.VERSION)
print('OpenGL: ', OpenGL.__version__)
