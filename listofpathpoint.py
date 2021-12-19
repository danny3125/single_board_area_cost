import cnc_input
#import img_index
import numpy as np
import matplotlib.pyplot as plt
import math
import torch
class input_handler:
    def __init__(self, jsonfilename):
        self.target_metrices = cnc_input.main(['-i', jsonfilename])
        self.X_all = []
        self.barrier_path = []
    def angle(self,v1):
        dx1 = v1[0]
        dy1 = v1[1]
        angle1 = math.atan2(dy1, dx1)
        angle1 = int(angle1 * 180 / math.pi)
        if angle1 < 0:
            angle1 = 360 + angle1
        return angle1
        
    def zig_zag_path(self,path_corners_index,barriers): #path corners index = [[start_corner_index, end_corner_index], ....] = array 2d (path_lengh,2)
        path_gazebo = []
        path_corners = []
        self.X_all = self.every_point()
        for index in path_corners_index:
            if len(index) == 2:
                path_corners.extend([self.X_all[index[0]],self.X_all[index[1]]])
            else:
                print(index)
                path_corners.extend([barriers[index[0]],barriers[index[1]],self.X_all[index[2]],self.X_all[index[3]]])
        data = np.array(path_corners)
        plt.plot(data[:, 0], data[:, 1])
        data_1 = np.array(self.X_all)
        data_1 = np.reshape(data_1,(int(len(self.X_all)/4),4,2))
        for rec in data_1:
            rec = np.concatenate((rec,[rec[0]]),axis= 0)
            plt.plot(rec[:, 0], rec[:, 1],color = 'red')
        data_barrier = np.array(barriers)
        data_barrier = np.reshape(data_barrier,(int(len(barriers)/4),4,2))
        for rec in data_barrier:
            rec = np.concatenate((rec,[rec[0]]),axis= 0)
            plt.plot(rec[:, 0], rec[:, 1],color = 'green')
        plt.show()
        
        for index in path_corners_index: #find the longer side => zig-zag to end point
            corner_num = index[0] % 4
            if (abs(self.X_all[index[0]][0] - self.X_all[index[1]][0])) > (abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])): #if longer side = horizon side = row side
                if corner_num == 0 :
                    y_way = range(0,int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),int(self.target_metrices[1]))
                    x_way_left = range(0,len(self.target_metrices[0][int(index[0] / 4)][0][0]),int(self.target_metrices[1]))
                    x_way_right = range(len(self.target_metrices[0][int(index[0] / 4)][0][0]),0,-int(self.target_metrices[1]))
                elif corner_num == 3:  # start = left down ,out = left up
                    y_way = range(0,-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])), -int(self.target_metrices[1]))
                    x_way_left = range(0,len(self.target_metrices[0][int(index[0] / 4)][0][0]),int(self.target_metrices[1]))
                    x_way_right = range(len(self.target_metrices[0][int(index[0] / 4)][0][0]),0,-int(self.target_metrices[1]))
                elif corner_num == 1: #start = right up, out = right down
                    y_way = range(0,int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),int(self.target_metrices[1]))
                    x_way_left = range(0,-len(self.target_metrices[0][int(index[0] / 4)][0][0]),-int(self.target_metrices[1]))
                    x_way_right = range(-len(self.target_metrices[0][int(index[0] / 4)][0][0]),0,int(self.target_metrices[1]))
                else: # corner_num == 2: #start = right down, out = right up
                    y_way = range(0,-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])), -int(self.target_metrices[1]))
                    x_way_left = range(0,-len(self.target_metrices[0][int(index[0] / 4)][0][0]),-int(self.target_metrices[1]))
                    x_way_right = range(-len(self.target_metrices[0][int(index[0] / 4)][0][0]),0,int(self.target_metrices[1]))
                # x_way_left = when the agent is on the left side , then it should move to the right side
                way_2 = x_way_left
                way_1 = y_way
                turn = 0
                for i in way_1:
                    turn += 1
                    for j in way_2:
                        path_gazebo.append([self.X_all[index[0]][0]+j,self.X_all[index[0]][1]+i])
                    if (turn % 2) == 1:
                        way_2 = x_way_right
                    else:
                        way_2 = x_way_left
            else:                                       # long side = straight side = column
                if corner_num == 0 : # lu -> ?
                    y_way = range(0,len(self.target_metrices[0][int(index[0] / 4)][0]),int(self.target_metrices[1]))
                    x_way_left = range(0,int(abs(self.X_all[index[0]][0] - self.X_all[index[1]][0])),int(self.target_metrices[1]))
                    x_way_right = range(int(abs(self.X_all[index[0]][0] - self.X_all[index[1]][0])),0,-int(self.target_metrices[1]))
                elif corner_num == 3:  # start = left down ,out = ?
                    y_way = range(len(self.target_metrices[0][int(index[0] / 4)][0]),0, -int(self.target_metrices[1]))
                    x_way_left = range(0,-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),-int(self.target_metrices[1]))
                    x_way_right = range(-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),0,int(self.target_metrices[1]))
                elif corner_num == 1: # start = right up, out = right down
                    y_way = range(0,-len(self.target_metrices[0][int(index[0] / 4)][0]),-int(self.target_metrices[1]))
                    x_way_left = range(0,int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),int(self.target_metrices[1]))
                    x_way_right = range(int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),0,-int(self.target_metrices[1]))
                else: #corner_num == 2: # start = right down, out = right up
                    y_way = range(0,-len(self.target_metrices[0][int(index[0] / 4)][0]), -int(self.target_metrices[1]))
                    x_way_left = range(0,-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),-int(self.target_metrices[1]))
                    x_way_right = range(-int(abs(self.X_all[index[0]][1] - self.X_all[index[1]][1])),0,int(self.target_metrices[1]))
                # x_way_left = when the agent is on the left side , then it should move to the right side
                way_2 = x_way_left
                way_1 = y_way
                turn = 0
                for i in way_1:
                    turn +=1
                    for j in way_2:
                        path_gazebo.append([self.X_all[index[0]][0]+j,self.X_all[index[0]][1]+i])
                    if (turn % 2) == 1:
                        way_2 = x_way_right
                    else:
                        way_2 = x_way_left
        return path_gazebo



    def A_walkonchip(self):
        allsteps = []
        for matrix in self.target_metrices:
            allsteps.extend((input_handler.sides_in_matrix(matrix[0], matrix[1], matrix[2])))
        return allsteps


    def sides_in_matrix(matrix, startpoint_x, startpoint_y):
        sides_of_matrix = ((startpoint_x, startpoint_y), (startpoint_x + len(matrix), startpoint_y) \
                               , (startpoint_x, startpoint_y + len(matrix[0]),
                                  (startpoint_x + len(matrix), startpoint_y + len(matrix[0]))))
        '''
        for i in range(1, len(matrix),6):
            for j in range(1,len(matrix[0]),6):
                walkpoints.append((startpoint_x + i, startpoint_y + j))
        return walkpoints'''
        return sides_of_matrix


    def package_points(self):
        X_all = []
        print(self.target_metrices[1])
        for matrix in self.target_metrices[0]:
            x = matrix[1]
            y = matrix[2]
            rectangle = matrix[0]
            if len(rectangle) > len(rectangle[0]):
                long_side = len(rectangle)
            else:
                long_side = len(rectangle[0])
            if (int(long_side / self.target_metrices[1]) % 2 == 0):
                corners = [[x, y], [x + len(rectangle), y],\
                           [x, y + len(rectangle[0])], [x + len(rectangle), y + len(rectangle[0])]]
            else:
                corners = [[x, y], [x + len(rectangle), y + len(rectangle[0])],\
                           [x + len(rectangle), y], [x, y + len(rectangle[0])]]
            X_all.append(corners)
        return X_all
    def point_scale(self):
        X_all = []
        for matrix in self.target_metrices[0]:
            x = matrix[1]
            y = matrix[2]
            rectangle = matrix[0]
            if len(rectangle) > len(rectangle[0]):
                long_side = len(rectangle)
            else:
                long_side = len(rectangle[0])   
            if(int(long_side / self.targetmatrices[1] % 2 == 0)):
                odd_even = 0
            else:
                odd_even = 1
            X_all.append([x,y,len(rectangle),len(rectangle[0]),odd_even])
        return X_all
    def every_point(self):
        self.X_all = []
        for matrix in self.target_metrices[0]:
            rectangle = matrix[0]
            x_lu = matrix[1]
            y_lu = matrix[2]
            x_ru = x_lu + len(rectangle[0])
            y_ru = y_lu
            x_ld = x_lu
            y_ld = y_lu + len(rectangle)
            x_rd = x_ru 
            y_rd = y_ld 
            
            self.X_all.extend([[x_lu,y_lu],[x_ru,y_ru],[x_rd,y_rd],[x_ld,y_ld]])
        return self.X_all
    def vec_euler(self,vector):
        return (vector[0]**2 + vector[1]**2)**0.5
    def extract_barrier_path(self):
        return self.barrier_path
    def barrier_detect(self, vector_barrier, euler_barrier, vector, euler):# size of all inputs : num_rec_corners x B x [value]
        self.barrier_path = []
        rewards = [0]*len(vector_barrier[0])
        vector = vector.tolist()
        euler = euler.tolist()
        for batch_num in range(len(vector_barrier[0])):
            single_map_barrier = vector_barrier[:,batch_num]
            single_map_barrier = single_map_barrier.tolist()
            single_map_euler = euler_barrier[:,batch_num]
            single_map_euler = single_map_euler.tolist()
            for i in range(0,len(vector_barrier),4):
                temp_vector = single_map_barrier[i:i+4]
                temp_euler = single_map_euler[i:i+4] # load four points of an object
                temp = []
                idxs = [j for j in range(i,i+4)]
                for vec,eul,k in zip(temp_vector,temp_euler,idxs):
                    temp.append([self.angle(vec),vec,eul,k]) # k for latter operation of index
                temp = sorted(temp, key = lambda temp : temp[0]) # small -> big
                temp_euler.sort()
                next_point_vec = self.angle(vector[batch_num])
                if next_point_vec >= temp[0][0] and next_point_vec <= temp[-1][0]:
                    if euler[batch_num] > temp_euler[0]:
                        # collision detected
                        if (abs(next_point_vec - temp[0][0])) < abs((next_point_vec - temp[-1][0])):
                            if batch_num == 0:
                                if temp[0][2] < temp[1][2]:
                                    self.barrier_path.append(temp[0][3])
                                    self.barrier_path.append(temp[1][3])
                                else:
                                    self.barrier_path.append(temp[1][3])
                                    self.barrier_path.append(temp[0][3])

                            vec_dis = [temp[0][1][0] - temp[1][1][0],temp[0][1][1] - temp[1][1][1]]
                            vec_dis = self.vec_euler(vec_dis)
                            vec_dis += self.vec_euler([vector[batch_num][0] - temp[1][1][0],vector[batch_num][1] - temp[1][1][1]])
                            vec_dis += self.vec_euler(temp[0][1])
                        else:
                            if batch_num == 0:
                                if temp[-1][2] < temp[-2][2]:
                                    self.barrier_path.append(temp[-1][3])
                                    self.barrier_path.append(temp[-2][3])
                                else:
                                    self.barrier_path.append(temp[-2][3])
                                    self.barrier_path.append(temp[-1][3])
                            vec_dis = [temp[-1][1][0] - temp[-2][1][0],temp[-1][1][1] - temp[-2][1][1]]
                            vec_dis = self.vec_euler(vec_dis)
                            vec_dis += self.vec_euler([vector[batch_num][0] - temp[-2][1][0],vector[batch_num][1] - temp[-2][1][1]])
                            vec_dis += self.vec_euler(temp[-1][1])    
                        rewards[batch_num]+= vec_dis - euler[batch_num]
                    else:
                        pass
                else:
                    pass
        rewards = torch.tensor(rewards)
        return rewards
    def outcorner_getout(self,rectangle_inf,B):# horizontal line = row
        feature = torch.Tensor([])
        area = torch.Tensor([])
        # is odd? is row?
        for inf in rectangle_inf:
            rectangle = self.target_metrices[0][int(inf)][0]
            area = torch.cat((area,torch.tensor(len(rectangle)*len(rectangle[0]))),0)
            index = 4*int(inf)
            corner = inf - int(inf)
            if len(rectangle) > len(rectangle[0]): # is column the long side?
                long_side = len(rectangle[0])
                if (int(long_side / self.target_metrices[1]) % 2 == 0): # take even times to spray 
                    if corner == 0: #is a left up corner, outcorner = left down
                        feature = torch.cat((feature,torch.Tensor([index + 3,index + 1,index + 2])),0) #append [outcorner,getout_corners]
                    elif corner == 0.25: # is a right up , out = right down
                        feature = torch.cat((feature,torch.Tensor([index + 2,index + 0,index + 3])),0)
                    elif corner == 0.5: #is a right down, out = right up
                        feature = torch.cat((feature,torch.Tensor([index + 1,index + 0,index + 3])),0)
                    else:               # is a left down, out = left up 
                        feature = torch.cat((feature,torch.Tensor([index + 1,index + 0,index + 2])),0)
                else:    #take odd time to spray
                    if corner == 0: #is a left up corner
                        feature = torch.cat((feature,torch.Tensor([index + 2,index + 1,index + 3])),0) #append [outcorner,getout_corners]
                    elif corner == 0.25: # is a right up 
                        feature = torch.cat((feature,torch.Tensor([index + 3,index + 0,index + 2])),0)
                    elif corner == 0.5: #is a right down
                        feature = torch.cat((feature,torch.Tensor([index + 0,index + 1,index + 3])),0)
                    else:               # is a left down
                        feature = torch.cat((feature,torch.Tensor([index + 1,index + 0,index + 2])),0)
                        
            else:
                long_side = len(rectangle) #row = long side
                if (int(long_side / self.target_metrices[1]) % 2 == 0): # take even times to spray   
                    if corner == 0: #is a left up corner
                        feature = torch.cat((feature,torch.Tensor([index + 1,index + 2,index + 3])),0)#append [outcorner,getout_corners]
                    elif corner == 0.25: # is a right up 
                        feature = torch.cat((feature,torch.Tensor([index + 0,index + 2,index + 3])),0)
                    elif corner == 0.5: #is a right down
                        feature = torch.cat((feature,torch.Tensor([index + 3,index + 0,index + 1])),0)
                    else:               # is a left down
                        feature = torch.cat((feature,torch.Tensor([index + 2,index + 0,index + 1])),0) 
                else:
                    if corner == 0: #is a left up corner
                        feature = torch.cat((feature,torch.Tensor([index + 2,index + 1,index + 3])),0) #append [outcorner,getout_corners]
                    elif corner == 0.25: # is a right up 
                        feature = torch.cat((feature,torch.Tensor([index + 3,index + 0,index + 2])),0)
                    elif corner == 0.5: #is a right down
                        feature = torch.cat((feature,torch.Tensor([index + 0,index + 1,index + 3])),0)
                    else:               # is a left down
                        feature = torch.cat((feature,torch.Tensor([index + 1,index + 0,index + 2])),0)
               
        feature = torch.reshape(feature,(B,3))                
        return feature,area    
            
        
'''
test = cnc_input.main(['-i', 'right_chip.json'])
print(test[9][0])
plt.imshow(test[9][0], cmap=plt.get_cmap('gray'))
A_walkonchip(test)
print(package_points(test))
'''