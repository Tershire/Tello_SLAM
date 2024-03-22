# propose_Q_and_R.py

#/////////////////////#
# IONLAB ISAE-SUPAERO #
# TELLO SLAM PROJECT  #
#/////////////////////#

# 2024 MAR 10
# Wonhee LEE

# reference: slambook


import numpy as np


mu = 0
sigma = 1.5

p = 1/(np.sqrt(2*np.pi))*np.exp(-0.5*((x - mu)/sigma)*((x - mu)/sigma))

