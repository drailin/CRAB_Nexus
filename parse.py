
import h5py
import pandas as pd
import numpy as np
import random
import os
import pickle

os.chdir('/Users/byrnesnk/nexus')
particles = pd.DataFrame(np.array(h5py.File('Pb210_2mm_100005.sim.h5')['MC']['particles']))

num = {}
for i in np.arange(0,100,1):
    num[i] = len(particles[(particles['particle_name'] == b'ie-' )&
                              (particles['event_id'] == i)]['final_y'])

pickle.dump( num, open( "run6.p", "wb" ) )
