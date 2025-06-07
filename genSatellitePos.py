#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun  7 11:59:56 2025

@author: paigejewell
"""

# Randomly Generate Satellite Positions Emulating Launch Vehicle Drop Off
# Leader satellite is left most
# 10 km spacing between kickoff
# 100 km box
# Inputs: number of satellites
# Outputs: sorted satellite positions

import numpy as np
import random

def genSatellitePos(numSats):
    xUncertainty = 5 #[km]
    yUncertainty = 10 #[km]
    zUncertainty = 10 #[km]
    
    xInit = 50 #[km]
    yInit = 0 #[km]
    zInit = 0 #[km]
    
    xDist = 10 #spacing [km]
    
    satPos = np.zeros((3, numSats))
    satPos[:,0] = [xInit, yInit, zInit]
    
    for i in range(1,numSats):
        # generate random uncertainity
        xU = random.uniform(-xUncertainty, xUncertainty)
        yU = random.uniform(-yUncertainty, yUncertainty)
        zU = random.uniform(-zUncertainty, zUncertainty)
        
        xPos = xInit - i*xDist + xU
        yPos = yInit + yU
        zPos = zInit + zU
        
        satPos[:,i] = [xPos, yPos, zPos]
    
    sortedIndices = np.argsort(-satPos[0])
    sortedSatPos = satPos[:, sortedIndices]
    
    return sortedSatPos

satsPos1 = genSatellitePos(5)