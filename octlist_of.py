
import cnc_input
import json
import matplotlib.pyplot as plt


def A_walkonchip(matrices):
    allsteps = []
    for matrix in matrices[0]:
       allsteps.extend((A_walkinmatrix(matrix[0],matrix[1],matrix[2])))
    return allsteps

def A_walkinmatrix(matrix,startpoint_x, startpoint_y ):
    walkpoints = []
    for j in range(1, len(matrix), 6):
        for i in range(1, len(matrix[0]),6):
            walkpoints.append((startpoint_x + i, startpoint_y + j))
    return walkpoints

def run():
    test = cnc_input.main(['-i','mother_board.json'])
    path = A_walkonchip(test)
    return path
